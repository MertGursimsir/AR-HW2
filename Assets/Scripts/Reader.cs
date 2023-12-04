using UnityEngine;
using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine.UI;

public class FileDataReader : MonoBehaviour
{
    public Button button;

    public string file1;
    public string file2;

    public int numOfRansacIterations = 10000;
    public float threshold = 0.1f;

    private bool drawLines = false;
    private List<Vector3> set1;
    private List<Vector3> set2;
    private List<Vector3> alignedSet2;

    private List<GameObject> drawnObjects = new List<GameObject>(); // Keep track of drawn objects

    void Start()
    {
        set1 = ReadFromFile(file1);
        set2 = ReadFromFile(file2);

        button.onClick.AddListener(ToggleDrawLines);
        SetButtonColor();

        //First step --> Find the rigid transformation
        //  Algorithm to use: RANSAC
        RansacResult result = Ransac(set1, set2);

        // Output the results
        Debug.Log("Rotation Matrix (R): \n" + result.rotationMatrix);
        Debug.Log("Translation Vector (T): " + result.translationVector);

        alignedSet2 = ApplyTransformation(set2, result.rotationMatrix, result.translationVector);
        DisplayPoints(alignedSet2, Color.blue, 3);
        DisplayPoints(set1, Color.green, 1);
        DisplayPoints(set2, Color.red, 2);

        DrawLineBetweenPoints();

    }

    void SetButtonColor()
    {
        ColorBlock colors = button.colors;
        colors.normalColor = drawLines ? Color.green : Color.red;
        button.colors = colors;

        button.image.color = colors.normalColor;
    }

    public List<Vector3> ReadFromFile(string fileName)
    {
        List<Vector3> points = new List<Vector3>();

        try
        {
            string path = Path.Combine(Application.dataPath, "Data", fileName);

            using (StreamReader sr = new StreamReader(path))
            {
                string firstLine = sr.ReadLine();
                int numPoints = int.Parse(firstLine);

                for (int i = 0; i < numPoints; i++)
                {
                    string line = sr.ReadLine();
                    string[] coordinates = line.Split(' ');

                    float x = float.Parse(coordinates[0]);
                    float y = float.Parse(coordinates[1]);
                    float z = float.Parse(coordinates[2]);
                    points.Add(new Vector3(x, y, z));
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError("Error reading file: " + e.Message);
        }

        return points;
    }

    void DisplayPoints(List<Vector3> points, Color color, int flag)
    {
        foreach (Vector3 point in points)
        {
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.position = point;
            Renderer sphereRenderer = sphere.GetComponent<Renderer>();
            sphereRenderer.material.color = color;
            drawnObjects.Add(sphere);
        }
    }

    void DrawLineBetweenPoints()
    {
        for (int i = 0; i < set2.Count; i++)
        {
            // Create a new empty GameObject to hold the LineRenderer component
            GameObject lineObject = new GameObject("Line" + i);
            drawnObjects.Add(lineObject);
            LineRenderer lineRenderer = lineObject.AddComponent<LineRenderer>();

            // Set the positions of the LineRenderer to the points in the first set
            lineRenderer.positionCount = 2;
            lineRenderer.SetPosition(0, set2[i]);
            lineRenderer.SetPosition(1, alignedSet2[i]);

            lineRenderer.startColor = Color.red;
            lineRenderer.endColor = Color.red;
            lineRenderer.startWidth = 0.1f;
            lineRenderer.endWidth = 0.1f;
        }
    }

    void ToggleDrawLines()
    {
        drawLines = !drawLines;
        SetButtonColor();
        ClearScreen();

        // If drawLines is true, draw lines and instantiate points
        if (drawLines)
        {
            DrawLineBetweenPoints();
            DisplayPoints(alignedSet2, Color.blue, 3);
        }
        // If drawLines is false, remove lines and points
        else
        {
            DisplayPoints(alignedSet2, Color.blue, 3);
            DisplayPoints(set1, Color.green, 1);
            DisplayPoints(set2, Color.red, 2);
        }
    }

    void ClearScreen()
    {
        // Destroy existing drawn objects
        foreach (var obj in drawnObjects)
        {
            Destroy(obj);
        }

        // Clear the list of drawn objects
        drawnObjects.Clear();
    }

    private struct RansacResult
    {
        public Matrix4x4 rotationMatrix;
        public Vector3 translationVector;
        public int inlierCount;
    }

    

    void SamplePoints(List<Vector3> points1, List<Vector3> points2, out List<Vector3> sampledPoints1, out List<Vector3> sampledPoints2)
    {
        sampledPoints1 = new List<Vector3>();
        sampledPoints2 = new List<Vector3>();

        // Randomly select three indices for each set
        List<int> indices1 = new List<int>();
        List<int> indices2 = new List<int>();

        while (indices1.Count < 3)
        {
            int index = UnityEngine.Random.Range(0, points1.Count);
            if (!indices1.Contains(index))
            {
                indices1.Add(index);
                sampledPoints1.Add(points1[index]);
            }
        }

        while (indices2.Count < 3)
        {
            int index = UnityEngine.Random.Range(0, points2.Count);
            if (!indices2.Contains(index))
            {
                indices2.Add(index);
                sampledPoints2.Add(points2[index]);
            }
        }
    }

    private RansacResult Ransac(List<Vector3> set1, List<Vector3> set2)
    {
        RansacResult bestResult = new RansacResult();

        for (int i = 0; i < numOfRansacIterations; i++)
        {
            // Randomly select three pairs of corresponding points
            List<Vector3> randomSet1 = new List<Vector3>();
            List<Vector3> randomSet2 = new List<Vector3>();
            SamplePoints(set1, set2, out randomSet1, out randomSet2);

            // Calculate the rigid transformation using Quaternion.FromToRotation
            RansacResult currentResult = CalculateRigidTransformation(randomSet1, randomSet2);

            // Count inliers based on the threshold
            int inlierCount = CountInliers(set1, set2, currentResult.rotationMatrix, currentResult.translationVector);

            // Update the best result if the current result has more inliers
            if (inlierCount > bestResult.inlierCount)
            {
                bestResult = currentResult;
                bestResult.inlierCount = inlierCount;
            }
        }

        return bestResult;
    }

    private RansacResult CalculateRigidTransformation(List<Vector3> set1, List<Vector3> set2)
    {
        // Calculate the centroid of each set
        Vector3 centroid1 = CalculateCentroid(set1);
        Vector3 centroid2 = CalculateCentroid(set2);

        // Subtract the centroids to center the sets around the origin
        List<Vector3> centeredSet1 = CenterPoints(set1, centroid1);
        List<Vector3> centeredSet2 = CenterPoints(set2, centroid2);

        // Calculate the rotation quaternion using Quaternion.FromToRotation
        Quaternion rotationQuaternion = Quaternion.FromToRotation(centeredSet2[1] - centeredSet2[0], centeredSet1[1] - centeredSet1[0]);

        // Convert quaternion to rotation matrix
        Matrix4x4 rotationMatrix = Matrix4x4.Rotate(rotationQuaternion);

        // Calculate the translation vector T
        Vector3 translationVector = centroid1 - rotationMatrix.MultiplyPoint3x4(centroid2);


        return new RansacResult
        {
            rotationMatrix = rotationMatrix,
            translationVector = translationVector
        };
    }

    // Center the points around the origin
    private List<Vector3> CenterPoints(List<Vector3> points, Vector3 centroid)
    {
        List<Vector3> centeredPoints = new List<Vector3>();

        foreach (Vector3 point in points)
        {
            centeredPoints.Add(point - centroid);
        }

        return centeredPoints;
    }

    private int CountInliers(List<Vector3> set1, List<Vector3> set2, Matrix4x4 rotationMatrix, Vector3 translationVector)
    {
        int inlierCount = 0;
        int min = (set1.Count < set2.Count) ? set1.Count : set2.Count;

        for (int i = 0; i < min; i++)
        {
            // Apply the rigid transformation to set2
            Vector3 transformedPoint = rotationMatrix.MultiplyPoint3x4(set2[i]) + translationVector;

            // Check if the transformed point is close to the corresponding point in set1
            if (Vector3.Distance(set1[i], transformedPoint) < threshold)
            {
                inlierCount++;
            }
        }

        return inlierCount;
    }

    // Calculate the centroid of a set of points
    private Vector3 CalculateCentroid(List<Vector3> points)
    {
        Vector3 sum = Vector3.zero;

        foreach (Vector3 point in points)
        {
            sum += point;
        }

        return sum / points.Count;
    }

    // Calculate the average of a set of points
    private Vector3 Average(List<Vector3> points)
    {
        return CalculateCentroid(points);
    }

    private List<Vector3> ApplyTransformation(List<Vector3> points, Matrix4x4 rotationMatrix, Vector3 translationVector)
    {
        List<Vector3> transformedPoints = new List<Vector3>();

        foreach (Vector3 point in points)
        {
            // Apply the rigid transformation to each point
            Vector3 transformedPoint = rotationMatrix.MultiplyPoint3x4(point) + translationVector;
            transformedPoints.Add(transformedPoint);
        }

        return transformedPoints;
    }
}