using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BallLauncher : MonoBehaviour
{
    // Start is called before the first frame update
    public Transform target;
    public GameObject ballPrefab;
    public float fireRate;
    public float ballLifeTime;
    public float speed;
    public float maxHorizontaDeviation;
    public float maxVericalDeviation;

    List<GameObject> ballPool;

    void Start()
    {
        ballPool = new List<GameObject>();
        Shoot();
    }

    void Shoot() 
    {
        GameObject ball = getBall();
        Vector3 targetPos = target.position;
        targetPos.x += Random.Range(-maxHorizontaDeviation, maxHorizontaDeviation);
        targetPos.y += Random.Range(-maxVericalDeviation, maxVericalDeviation);
        ball.GetComponent<Rigidbody>().velocity = (targetPos - transform.position).normalized * speed;
        Invoke("Shoot", fireRate);
    }

    GameObject getBall() 
    {
        GameObject ball;
        int i = 0;
        while (i < ballPool.Count) {
            if (!ballPool[i].activeInHierarchy) {
                ball = ballPool[i];
                ball.transform.position = transform.position;
                ball.SetActive(true);
                return ball;
            }
            i++;
        }
        return newBall();
    }

    GameObject newBall() 
    {
        GameObject ball = Instantiate(ballPrefab);
        ball.GetComponent<Ball>().lifeTime = ballLifeTime;
        ballPool.Add(ball);
        return ball;
    }
    // Update is called once per frame
    void Update()
    {
        
    }
}
