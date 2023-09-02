using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class cube_motion : MonoBehaviour
{
    bool pressed = false;
    public bool cube_move = false;
    Vector3 offset;
    Vector3 v = new Vector3(0, 0, 0);   // velocity
    Vector3 w = new Vector3(0, 0, 0); // angular velocity
    float dt = 0.01f;//0.015

    Matrix4x4 I_ref;            // reference inertia
    float mass = 1.0f;                 // mass

    float linear_decay = 0.99f;                // for velocity decay
    float angular_decay = 0.98f;

    Vector3 external = new Vector3(0, -9.8f, 0);
    Vector3 buoyancy = Vector3.zero;
    Vector3 tao = Vector3.zero;

    // Start is called before the first frame update
    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;

        /*
        float m = 1;
        mass = 0;
        for (int i = 0; i < vertices.Length; i++)
        {
            mass += m;
            float diag = m * vertices[i].sqrMagnitude;
            I_ref[0, 0] += diag;
            I_ref[1, 1] += diag;
            I_ref[2, 2] += diag;
            I_ref[0, 0] -= m * vertices[i][0] * vertices[i][0];
            I_ref[0, 1] -= m * vertices[i][0] * vertices[i][1];
            I_ref[0, 2] -= m * vertices[i][0] * vertices[i][2];
            I_ref[1, 0] -= m * vertices[i][1] * vertices[i][0];
            I_ref[1, 1] -= m * vertices[i][1] * vertices[i][1];
            I_ref[1, 2] -= m * vertices[i][1] * vertices[i][2];
            I_ref[2, 0] -= m * vertices[i][2] * vertices[i][0];
            I_ref[2, 1] -= m * vertices[i][2] * vertices[i][1];
            I_ref[2, 2] -= m * vertices[i][2] * vertices[i][2];
        }
        I_ref[3, 3] = 1;*/
        float i = (mass * 1 * 1) / 12;

        I_ref = new Matrix4x4();
        I_ref[0, 0] = i;
        I_ref[1, 1] = i;
        I_ref[2, 2] = i;
        I_ref[3, 3] = 1;
    }

    // Update is called once per frame
    void Update()
    {
        Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
        Matrix4x4 I = R * I_ref * R.transpose;
        external = new Vector3(0, -9.8f, 0) + buoyancy;

        v += external / mass * dt;
        w += (Vector3)(I.inverse * tao * dt);
        v *= linear_decay;
        w *= angular_decay;
        Vector3 x = transform.position;
        x += v * dt;
        Quaternion q = transform.rotation;
        Quaternion t = new Quaternion(dt / 2 * w[0], dt / 2 * w[1], dt / 2 * w[2], 0);
        t = t * q;
        q = new Quaternion(t[0] + q[0], t[1] + q[1], t[2] + q[2], t[3] + q[3]);
        transform.position = x;
        transform.rotation = q;

        if (Input.GetMouseButtonDown(0))
        {
            pressed = true;
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Vector3.Cross(ray.direction, transform.position - ray.origin).magnitude < 0.8f) cube_move = true;
            else cube_move = false;
            offset = Input.mousePosition - Camera.main.WorldToScreenPoint(transform.position);
        }
        if (Input.GetMouseButtonUp(0))
        {
            pressed = false;
            cube_move = false;
        }

        if (pressed)
        {
            if (cube_move)
            {
                Vector3 mouse = Input.mousePosition;
                mouse -= offset;
                mouse.z = Camera.main.WorldToScreenPoint(transform.position).z;
                Vector3 p = Camera.main.ScreenToWorldPoint(mouse);
                p.y = transform.position.y;
                transform.position = p;
            }
            else
            {
                //float h = 2.0f * Input.GetAxis("Mouse X");
                //Camera.main.transform.RotateAround(new Vector3(0, 0, 0), Vector3.up, h);
            }
        }
        
    }

    //力位置position，力f，世界坐标，需要转换
    public void addForce(Vector3 p, Vector3 f)
    {
        Vector3 local_p = transform.InverseTransformPoint(p);
        Vector3 local_f = transform.InverseTransformDirection(f);
        Vector3 r = p - transform.position;
        Vector3 Rr = Matrix4x4.Rotate(transform.rotation) * (p - transform.position);
        buoyancy += f;//Vector3.Dot(f, r) * r;// Vector3.Dot(f, Rr) * Rr.normalized;
        tao += Vector3.Cross(r, f);
    }
    public void clearForce()
    {
        buoyancy = Vector3.zero;
        tao = Vector3.zero;
    }
}
