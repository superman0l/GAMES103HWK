using UnityEngine;
using System.Collections;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net.Sockets;
using System.Net;

public class wave_motion : MonoBehaviour
{
    int size = 100;
    float rate = 0.005f;
    float gamma = 0.004f;//0.004
    float damping = 0.996f;
    float[,] old_h;
    float[,] low_h;
    float[,] vh;
    float[,] b;

    bool[,] cg_mask;
    float[,] cg_p;
    float[,] cg_r;
    float[,] cg_Ap;
    bool tag = true;

    Vector3 cube_v = Vector3.zero;
    Vector3 cube_w = Vector3.zero;

    int cube_li = 101, cube_ui = -1, cube_lj = 101, cube_uj = -1;
    int block_li = 101, block_ui = -1, block_lj = 101, block_uj = -1;

    static System.Random rand;

    public float rho = 1f;
    float g = 9.8f;
    float dA = 0.01f;

    HashSet<(int, int, Vector3)> set = new HashSet<(int, int, Vector3)>();

    GameObject Block, Cube;

    // Use this for initialization
    void Start()
    {
        Block = GameObject.Find("Block");
        Cube = GameObject.Find("Cube");

        rand = new System.Random();
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        mesh.Clear();

        Vector3[] X = new Vector3[size * size];

        for (int i = 0; i < size; i++)
            for (int j = 0; j < size; j++)
            {
                X[i * size + j].x = i * 0.1f - size * 0.05f;
                X[i * size + j].y = 0;
                X[i * size + j].z = j * 0.1f - size * 0.05f;
            }

        int[] T = new int[(size - 1) * (size - 1) * 6];
        int index = 0;
        for (int i = 0; i < size - 1; i++)
            for (int j = 0; j < size - 1; j++)
            {
                T[index * 6 + 0] = (i + 0) * size + (j + 0);
                T[index * 6 + 1] = (i + 0) * size + (j + 1);
                T[index * 6 + 2] = (i + 1) * size + (j + 1);
                T[index * 6 + 3] = (i + 0) * size + (j + 0);
                T[index * 6 + 4] = (i + 1) * size + (j + 1);
                T[index * 6 + 5] = (i + 1) * size + (j + 0);
                index++;
            }
        mesh.vertices = X;
        mesh.triangles = T;
        mesh.RecalculateNormals();

        low_h = new float[size, size];
        old_h = new float[size, size];
        vh = new float[size, size];
        b = new float[size, size];

        cg_mask = new bool[size, size];
        cg_p = new float[size, size];
        cg_r = new float[size, size];
        cg_Ap = new float[size, size];

        for (int i = 0; i < size; i++)
            for (int j = 0; j < size; j++)
            {
                low_h[i, j] = 99999;
                old_h[i, j] = 0;
                vh[i, j] = 0;
            }
    }

    void A_Times(bool[,] mask, float[,] x, float[,] Ax, int li, int ui, int lj, int uj)
    {
        for (int i = li; i <= ui; i++)
            for (int j = lj; j <= uj; j++)
                if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                {
                    Ax[i, j] = 0;
                    if (i != 0) Ax[i, j] -= x[i - 1, j] - x[i, j];
                    if (i != size - 1) Ax[i, j] -= x[i + 1, j] - x[i, j];
                    if (j != 0) Ax[i, j] -= x[i, j - 1] - x[i, j];
                    if (j != size - 1) Ax[i, j] -= x[i, j + 1] - x[i, j];
                }
    }

    float Dot(bool[,] mask, float[,] x, float[,] y, int li, int ui, int lj, int uj)
    {
        float ret = 0;
        for (int i = li; i <= ui; i++)
            for (int j = lj; j <= uj; j++)
                if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                {
                    ret += x[i, j] * y[i, j];
                }
        return ret;
    }

    void Conjugate_Gradient(bool[,] mask, float[,] b, float[,] x, int li, int ui, int lj, int uj)
    {
        //Solve the Laplacian problem by CG.
        A_Times(mask, x, cg_r, li, ui, lj, uj);

        for (int i = li; i <= ui; i++)
            for (int j = lj; j <= uj; j++)
                if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                {
                    cg_p[i, j] = cg_r[i, j] = b[i, j] - cg_r[i, j];
                }

        float rk_norm = Dot(mask, cg_r, cg_r, li, ui, lj, uj);

        for (int k = 0; k < 128; k++)
        {
            if (rk_norm < 1e-10f) break;
            A_Times(mask, cg_p, cg_Ap, li, ui, lj, uj);
            float alpha = rk_norm / Dot(mask, cg_p, cg_Ap, li, ui, lj, uj);

            for (int i = li; i <= ui; i++)
                for (int j = lj; j <= uj; j++)
                    if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                    {
                        x[i, j] += alpha * cg_p[i, j];
                        cg_r[i, j] -= alpha * cg_Ap[i, j];
                    }

            float _rk_norm = Dot(mask, cg_r, cg_r, li, ui, lj, uj);
            float beta = _rk_norm / rk_norm;
            rk_norm = _rk_norm;

            for (int i = li; i <= ui; i++)
                for (int j = lj; j <= uj; j++)
                    if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                    {
                        cg_p[i, j] = cg_r[i, j] + beta * cg_p[i, j];
                    }
        }

    }

    void Shallow_Wave(float[,] old_h, float[,] h, float[,] new_h)
    {
        //Step 1:
        //TODO: Compute new_h based on the shallow wave model.
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                float h_sum = 0;
                for (int k = -1; k <= 1; k += 2)
                {
                    if (i + k < 0 || i + k >= size) h_sum += h[i, j];
                    else h_sum += h[i + k, j];
                    if (j + k < 0 || j + k >= size) h_sum += h[i, j];
                    else h_sum += h[i, j + k];
                }
                new_h[i, j] = h[i, j] + (h[i, j] - old_h[i, j]) * damping + (h_sum - 4 * h[i, j]) * rate;
            }
        }
        //Step 2: Block->Water coupling
        //TODO: for block 1, calculate low_h.
        //TODO: then set up b and cg_mask for conjugate gradient.
        //TODO: Solve the Poisson equation to obtain vh (virtual height).

        //TODO: for block 2, calculate low_h.
        //TODO: then set up b and cg_mask for conjugate gradient.
        //TODO: Solve the Poisson equation to obtain vh (virtual height).
        RaycastHit hitInfo;
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] X = mesh.vertices;
        cube_li = 101; cube_ui = -1; cube_lj = 101; cube_uj = -1;
        block_li = 101; block_ui = -1; block_lj = 101; block_uj = -1;
        set.Clear();
        Cube.GetComponent<cube_motion>().clearForce();
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                Vector3 t0 = X[i * size + j], dir = new Vector3(0, 1, 0);
                t0.y = -30; t0 += transform.position;
                if (Physics.Raycast(t0, dir, out hitInfo) && (hitInfo.collider.gameObject.name == "Cube" || hitInfo.collider.gameObject.name == "Block"))
                {
                    low_h[i, j] = hitInfo.point.y - transform.position.y;
                    b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
                    cg_mask[i, j] = true;
                    
                    if (hitInfo.collider.gameObject.name == "Cube")
                    {
                        cube_li = i < cube_li ? i : cube_li;
                        cube_ui = i > cube_ui ? i : cube_ui;
                        cube_lj = j < cube_lj ? j : cube_lj;
                        cube_uj = j > cube_uj ? j : cube_uj;
                        (int, int, Vector3) t = (i, j, hitInfo.point);
                        set.Add(t);
                        //Vector3 force = rho * g * dA * (h[i, j] - new_h[i,j]) * new Vector3(0, 1f, 0);
                        //Cube.GetComponent<cube_motion>().addForce(hitInfo.point, force);
                    }
                    else
                    {
                        block_li = i < block_li ? i : block_li;
                        block_ui = i > block_ui ? i : block_ui;
                        block_lj = j < block_lj ? j : block_lj;
                        block_uj = j > block_uj ? j : block_uj;
                    }
                }
                else
                {
                    vh[i, j] = 0; cg_mask[i, j] = false;
                }
            }
        }

        Conjugate_Gradient(cg_mask, b, vh, cube_li, cube_ui, cube_lj, cube_uj);
        Conjugate_Gradient(cg_mask, b, vh, block_li, block_ui, block_lj, block_uj);
        //TODO: Diminish vh.

        //TODO: Update new_h by vh.
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                float v_sum = 0;
                for (int k = -1; k <= 1; k += 2)
                {
                    if (i + k < 0 || i + k >= size) v_sum += vh[i, j];
                    else v_sum += vh[i + k, j];
                    if (j + k < 0 || j + k >= size) v_sum += vh[i, j];
                    else v_sum += vh[i, j + k];
                }
                new_h[i, j] += (v_sum - 4 * vh[i, j]) * rate * gamma;
            }
        }

        foreach (var item in set)
        {
            Vector3 force = Vector3.zero;
            force = rho * g * dA * (vh[item.Item1, item.Item2]) * new Vector3(0,0.01f,0);
            //Vector3 force = Vector3.zero;
            //if (h[item.Item1, item.Item2] > new_h[item.Item1, item.Item2]) force = rho * g * dA * (h[item.Item1, item.Item2] - new_h[item.Item1, item.Item2]) * new Vector3(0, 100f, 0);
            Cube.GetComponent<cube_motion>().addForce(item.Item3, force);
        }

        //Step 3
        //TODO: old_h <- h; h <- new_h;
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                old_h[i, j] = h[i, j];
                h[i, j] = new_h[i, j];
            }
        }
        //Step 4: Water->Block coupling.
        //More TODO here.
    }


    // Update is called once per frame
    void Update()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] X = mesh.vertices;
        float[,] new_h = new float[size, size];
        float[,] h = new float[size, size];

        //TODO: Load X.y into h.
        for (int i = 0; i < X.Length; i++)
        {
            int x = i / size, y = i % size;
            h[x, y] = X[i][1];
        }

        if (Input.GetKeyDown("r"))
        {
            //TODO: Add random water.
            int x = (int)(rand.NextDouble() * size), y = (int)(rand.NextDouble() * size);
            h[x, y] += (float)rand.NextDouble() * 0.5f;
        }

        for (int l = 0; l < 8; l++)
        {
            Shallow_Wave(old_h, h, new_h);
        }

        //TODO: Store h back into X.y and recalculate normal.
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                X[i * size + j].y = h[i, j];
            }
        }
        mesh.vertices = X;
        mesh.RecalculateNormals();
    }
}
