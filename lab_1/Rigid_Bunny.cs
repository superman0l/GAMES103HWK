using UnityEngine;
using System.Collections;
using System;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0); // angular velocity
    Vector3 g			= new Vector3(0, -9.8f, 0);

    float mass;					// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				

	public float restitution 	= 0.5f;                 // for bound
    public float friction		= 0.2f;					// fric

    // Use this for initialization
    void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}

    Matrix4x4 sub(Matrix4x4 m1, Matrix4x4 m2)
    {
        Matrix4x4 res = Matrix4x4.zero;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                res[i, j] = m1[i, j] - m2[i, j];
        return res;
    }

    Matrix4x4 Matrix_miltiply(Matrix4x4 a, float b)
    {
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                a[i, j] *= b;
            }
        }
        return a;
    }

    Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
        Matrix4x4 I = R * I_ref * R.transpose;
		Vector3 dv_sum = new Vector3(0, 0, 0);
        Vector3 dw_sum = new Vector3(0, 0, 0);
        int coli_n = 0;
        for (int i = 0; i < vertices.Length; i++)
		{
			Vector3 Rri = R * vertices[i];
            Vector3 p_i = transform.position + Rri;
            if (Vector3.Dot((p_i - P), N) < 0)//穿过，碰撞发生
			{
				Vector3 vex_v = v + Vector3.Cross(w, Rri);
				if (Vector3.Dot(vex_v, N) < 0)//速度
				{
					coli_n++;
					Vector3 velocity_n = Vector3.Dot(vex_v, N) * N;
                    Vector3 velocity_t = vex_v - velocity_n;
					float a = Math.Max(0, 1.0f - friction * (1.0f + restitution) * velocity_n.magnitude / velocity_t.magnitude);
                    velocity_n = -restitution * velocity_n;
					velocity_t *= a;
					Vector3 n_o = velocity_n + velocity_t - vex_v;

                    Matrix4x4 k;
					Matrix4x4 Rrstar = Get_Cross_Matrix(Rri);
					k = sub(Matrix_miltiply(Matrix4x4.identity, 1.0f / mass), Rrstar * I.inverse * Rrstar);
					Vector3 j = k.inverse * n_o;
					dv_sum += j / mass;
                    Vector3 deltaw = I.inverse * Vector3.Cross(Rri, j);
                    dw_sum += deltaw;
                }
			}
		}
		if (coli_n == 0) return;
		v += dv_sum / coli_n;
		w += dw_sum / coli_n;
    }

    // Update is called once per frame
    void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			launched=true;
		}
        if (launched == false) return;
        // Part I: Update velocities
        // 只计算重力？不更新角速度

        v += g * dt;
		v *= linear_decay;
		w *= angular_decay;

		// Part II: Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

		// Part III: Update position & orientation
		//Update linear status
		Vector3 x    = transform.position;
		x += v * dt;
		//Update angular status
		Quaternion q = transform.rotation;
		Quaternion t = new Quaternion(dt / 2 * w[0], dt / 2 * w[1], dt / 2 * w[2], 0);
		t = t * q;
		q = new Quaternion(t[0] + q[0], t[1] + q[1], t[2] + q[2], t[3] + q[3]);

		// Part IV: Assign to the object
		transform.position = x;
		transform.rotation = q;
	}
}
