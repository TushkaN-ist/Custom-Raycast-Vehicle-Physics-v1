using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TireSFX : MonoBehaviour
{
	public CustomWheelCollider wheel;
	public AudioClip[] skidLvl;
	public AnimationCurve skidMarge;
	public float maxSkidForce=60;
	public float smooth=3;
	AudioSource[] sources;
    // Start is called before the first frame update
    void Start()
    {
	    sources=new AudioSource[skidLvl.Length];
	    AudioSource src;
	    for (int i = 0; i < sources.Length; i++) {
	    	src = gameObject.AddComponent<AudioSource>();
	    	src.loop=true;
	    	src.playOnAwake=false;
	    	src.clip=skidLvl[i];
	    	src.spatialBlend=1;
	    	//src.Play();
	    	sources[i] = src;
	    }
    }
	public float pow=4;
	Vector3 vel;
    // Update is called once per frame
    void Update()
	{
		vel = Vector3.Lerp(wheel.wheelLocalVelocity,vel,Time.deltaTime/smooth);
		vel.y=0;
		float itemID=1;
		foreach (var item in sources)
	    {
			item.volume = Mathf.Pow(Mathf.Clamp01(1f-Mathf.Abs(skidMarge.Evaluate(vel.magnitude/maxSkidForce*sources.Length)-itemID)),pow);
			if (item.volume<=0.1f)
			{
				if (item.isPlaying)
					item.Stop();
			}else{
				
				if (!item.isPlaying)
					item.Play();
			}
			itemID++;
		}
    }
}
