using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM : MonoBehaviour
{
	float dt 			= 0.003f;//0.002
    float mass 			= 1;
	float stiffness_0	= 20000.0f;//15000
    float stiffness_1 	= 5000.0f;//7500
    float damp			= 0.999f;

    float friction = 0.2f;
    float restitution = 0.5f;

    Vector3 gravity = new Vector3(0, -9.8f, 0);

	int[] 		Tet;
	int tet_number;			//The number of c

	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	int number;				//The number of vertices
    int[][] neighbors;      // neighbors of vertices

    Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[]   V_sum;
	int[]		V_num;

	SVD svd = new SVD();

    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
    		string fileContent = File.ReadAllText("Assets/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
    		tet_number=int.Parse(Strings[0]);
        	Tet = new int[tet_number*4];

    		for(int tet=0; tet<tet_number; tet++)
    		{
				Tet[tet*4+0]=int.Parse(Strings[tet*5+4])-1;
				Tet[tet*4+1]=int.Parse(Strings[tet*5+5])-1;
				Tet[tet*4+2]=int.Parse(Strings[tet*5+6])-1;
				Tet[tet*4+3]=int.Parse(Strings[tet*5+7])-1;
			}
    	}
    	{
			string fileContent = File.ReadAllText("Assets/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
       		for(int i=0; i<number; i++)
       		{
       			X[i].x=float.Parse(Strings[i*5+5])*0.4f;
       			X[i].y=float.Parse(Strings[i*5+6])*0.4f;
       			X[i].z=float.Parse(Strings[i*5+7])*0.4f;
       		}
    		//Centralize the model.
	    	Vector3 center=Vector3.zero;
	    	for(int i=0; i<number; i++)		center+=X[i];
	    	center=center/number;
	    	for(int i=0; i<number; i++)
	    	{
	    		X[i]-=center;
	    		float temp=X[i].y;
	    		X[i].y=X[i].z;
	    		X[i].z=temp;
	    	}
		}
        /*tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);*/


        //Create triangle mesh.
       	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];

        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }

        int[] triangles = new int[tet_number*12];
        for(int t=0; t<tet_number*4; t++)
        {
        	triangles[t*3+0]=t*3+0;
        	triangles[t*3+1]=t*3+1;
        	triangles[t*3+2]=t*3+2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();


		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

        //TODO: Need to allocate and assign inv_Dm
        inv_Dm = new Matrix4x4[tet_number]; //初始的位置矩阵
        for (int tet = 0; tet < tet_number; tet++)
            inv_Dm[tet] = Build_Edge_Matrix(tet).inverse;

        int[,] links = new int[number, number];
		for (int tet = 0; tet < tet_number; tet++)
		{
			links[Tet[tet*4+0], Tet[tet*4+1]] = 1;
			links[Tet[tet*4+0], Tet[tet*4+2]] = 1;
			links[Tet[tet*4+0], Tet[tet*4+3]] = 1;
			links[Tet[tet*4+1], Tet[tet*4+0]] = 1;
			links[Tet[tet*4+1], Tet[tet*4+2]] = 1;
			links[Tet[tet*4+1], Tet[tet*4+3]] = 1;
			links[Tet[tet*4+2], Tet[tet*4+0]] = 1;
			links[Tet[tet*4+2], Tet[tet*4+1]] = 1;
			links[Tet[tet*4+2], Tet[tet*4+3]] = 1;
			links[Tet[tet*4+3], Tet[tet*4+0]] = 1;
			links[Tet[tet*4+3], Tet[tet*4+1]] = 1;
			links[Tet[tet*4+3], Tet[tet*4+2]] = 1;
		}

		neighbors = new int[number][];
		for (int i = 0; i < number; i++)
		{
			for (int j = i + 1; j < number; j++)
			{
				if (links[i, j] == 1)
				{
					V_num[i] += 1;
					V_num[j] += 1;
				}
			}
			neighbors[i] = new int[V_num[i]];
		}

        for (int i = 0; i < X.Length; i++) V_num[i] = 0;
		for (int i = 0; i < number - 1; i++)
		{
			for (int j = i + 1; j < number; j++)
			{
				if (links[i, j] == 1)
				{
					neighbors[i][V_num[i]] = j;
					neighbors[j][V_num[j]] = i;
                    V_num[i]++;
                    V_num[j]++;
				}	
			}
		}
    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 ret=Matrix4x4.zero;
        //TODO: Need to build edge matrix here.
        Vector3 x10 = X[Tet[tet * 4 + 1]] - X[Tet[tet * 4 + 0]],
            x20 = X[Tet[tet * 4 + 2]] - X[Tet[tet * 4 + 0]],
            x30 = X[Tet[tet * 4 + 3]] - X[Tet[tet * 4 + 0]];
        ret.SetColumn(0, x10);
        ret.SetColumn(1, x20);
        ret.SetColumn(2, x30);
        ret[3, 3] = 1;
		return ret;
    }

    Matrix4x4 Matrix_miltiply(Matrix4x4 a, float b)
    {
        Matrix4x4 A = Matrix4x4.zero;
        for (int i = 0; i < 4; i++) A.SetColumn(i, b * a.GetColumn(i));
        return A;
    }

    Matrix4x4 sub(Matrix4x4 m1, Matrix4x4 m2)
    {
        Matrix4x4 A = Matrix4x4.zero;
        for (int i = 0; i < 4; i++) A.SetColumn(i, m1.GetColumn(i) - m2.GetColumn(i));
        return A;
    }

    Matrix4x4 add(Matrix4x4 m1, Matrix4x4 m2)
    {
        Matrix4x4 A = Matrix4x4.zero;
        for (int i = 0; i < 4; i++) A.SetColumn(i, m1.GetColumn(i) + m2.GetColumn(i));
        return A;
    }

    float Tr(Matrix4x4 m)
    {
        return m[0, 0] + m[1, 1] + m[2, 2];
    }

    void Smooth_V()
    {
        float alpha = 0.9f;

        for (int i = 0; i < number; i++)
        {
            V_sum[i] = Vector3.zero;
            for (int j = 0; j < neighbors[i].Length; j++)
            {
                V_sum[i] += V[neighbors[i][j]];
            }
            //V[i] = alpha * V[i] + (1 - alpha) * V_sum[i] / V_num[i];
        }
    }

    Matrix4x4 Get_P_StVK(Matrix4x4 F)
    {
        //TODO: Green Strain
        Matrix4x4 G = Matrix_miltiply(sub(F.transpose * F, Matrix4x4.identity), 0.5f);
        //TODO: Second PK Stress
        Matrix4x4 S = add(Matrix_miltiply(G, 2 * stiffness_1), Matrix_miltiply(Matrix4x4.identity, stiffness_0 * Tr(G)));
        Matrix4x4 P = F * S;
        return P;
    }

    Matrix4x4 Get_P_NeoHookean(Matrix4x4 F)
    {
        Matrix4x4 U = Matrix4x4.zero, S = Matrix4x4.zero, V = Matrix4x4.zero;
        svd.svd(F, ref U, ref S, ref V);
        Matrix4x4 diag = Matrix4x4.identity;
        
        float i1 = Mathf.Pow(S[0, 0], 2) + Mathf.Pow(S[1, 1], 2) + Mathf.Pow(S[2, 2], 2);
        float i3 = Mathf.Pow(S[0, 0], 4) + Mathf.Pow(S[1, 1], 4) + Mathf.Pow(S[2, 2], 4);
        for(int i = 0; i < 3; i++)
        {
            diag[i, i] = stiffness_0 * (2 * S[i, i] * (3 * i3 - 2 * Mathf.Pow(S[i, i], 2) * i1)) / (3 * Mathf.Pow(i3, 4.0f / 3))
                + stiffness_1 * (-2 * Mathf.Pow(S[i, i], 3)) / Mathf.Pow(i3, 3.0f / 2);
            //diag[i, i] = stiffness_0 * 0.5f * (4 * S[i, i] * (i1 - 3)) + stiffness_1 * 0.25f * (2 * S[i, i] * (i1 - 2) - 2 * Mathf.Pow(S[i, i], 3));
        }
        return U *diag*V.transpose;
    }

    void _Update()
    {
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y+=0.2f;
    	}

    	for(int i=0 ;i<number; i++)
    	{
            //TODO: Add gravity to Force.
            Force[i] = gravity * mass;
    	}

    	for(int tet=0; tet<tet_number; tet++)
    	{
            //TODO: Deformation Gradient
            Matrix4x4 F = Build_Edge_Matrix(tet) * inv_Dm[tet];
            F[3, 3] = 1;
            Matrix4x4 P = Get_P_StVK(F);
            //Matrix4x4 P = Get_P_NeoHookean(F);
            //TODO: Elastic Force
            Matrix4x4 force = Matrix_miltiply(P * inv_Dm[tet].transpose, -1.0f / (6 * inv_Dm[tet].determinant));
            force[3, 3] = 1;
            Force[Tet[tet * 4 + 1]] += (Vector3)force.GetColumn(0);
            Force[Tet[tet * 4 + 2]] += (Vector3)force.GetColumn(1);
            Force[Tet[tet * 4 + 3]] += (Vector3)force.GetColumn(2);
            Force[Tet[tet * 4 + 0]] += (Vector3)(-force.GetColumn(0) - force.GetColumn(1) - force.GetColumn(2));
        }

        Smooth_V();

        for (int i=0; i<number; i++)
    	{
            //TODO: Update X and V here.
            V[i] += Force[i] / mass * dt;
            V[i] *= damp;
            V[i] = (V_num[i] * V[i] + V_sum[i]) / (2 * V_num[i]);
            X[i] += V[i] * dt;
            //TODO: (Particle) collision with floor.
            Vector3 P = new Vector3(0, -3.0f, 0), N = new Vector3(0, 1.0f, 0);
            if (Vector3.Dot((X[i] - P), N) < 0)//穿过，碰撞发生
            {
                X[i] -= Vector3.Dot((X[i] - P), N) * N;
                if (Vector3.Dot(V[i], N) < 0) //速度
                {
                    Vector3 velocity_n = Vector3.Dot(V[i], N) * N;
                    Vector3 velocity_t = V[i] - velocity_n;
                    float a = Math.Max(0, 1.0f - friction * (1.0f + restitution) * velocity_n.magnitude / velocity_t.magnitude);
                    velocity_n = -restitution * velocity_n;
                    velocity_t *= a;
                    V[i] = velocity_n + velocity_t;
                }
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
    	for(int l=0; l<10; l++)
    		 _Update();

    	// Dump the vertex array for rendering.
    	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.RecalculateNormals ();
    }
}
