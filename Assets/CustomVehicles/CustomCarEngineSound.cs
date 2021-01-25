using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CustomCarEngineSound : MonoBehaviour
{
	public float rpm=0,normalizeRPM=4000;
	public Sound idle,low,mid,high;
	public CustomCarEngine engine;
	public bool engineRPM;
	
	// This function is called when the script is loaded or a value is changed in the inspector (Called in the editor only).
	protected void OnValidate()
	{
	}
	
    // Start is called before the first frame update
	void Start()
	{
		idle.Init(gameObject);
		low.Init(gameObject);
		mid.Init(gameObject);
		high.Init(gameObject);
	}
    
    // Update is called once per frame
    void Update()
	{
		if (engineRPM)
			rpm = engine.rpm;
		idle.Update(rpm);
		low.Update(rpm);
		mid.Update(rpm);
		high.Update(rpm);
    }
    
	[System.Serializable]
	public struct Sound{
		public bool enabled;
		public AudioClip clip;
		AudioSource src;
		public float min,max;
		public AnimationCurve pith,volume;
		
		public void Init(GameObject gameObject)
		{
			if (src==null)
				src = gameObject.AddComponent<AudioSource>();
			src.loop=true;
			src.clip = clip;
			src.spatialBlend = 1;
			src.playOnAwake=false;
			src.Play();
		}
		
		public void Destroy(){
			MonoBehaviour.Destroy(src);
		}
		
		public void EditorUpdate(){
			
		}
		
		public void Update(float rpm){
			if (!enabled){
				src.volume=0;
				return;
			}
			float range = (rpm-min)/(max-min);
			src.volume = volume.Evaluate(range);
			src.pitch = pith.Evaluate(range);
		}
	}
    
}
