using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CustomCarGearBox : MonoBehaviour
{
	[Range(0,1),SerializeField]
	float clutch=0;
	
	[SerializeField]
	int gearFirst=1;
	public float topGear=3.9f;
	public float[] gearRatio=new float[]{-3.53f,0,3.67f,2.1f,1.36f,1f,0.82f};
	
	public int gear;
	
	public void SetGear(int id){
		gear=id;
	}
	public bool isNeitrale{
		get{
			return (gear+gearFirst)==gearFirst;
		}
	}
	public float GetGear(){
		return isNeitrale?1:gearRatio[Mathf.Clamp(gear+gearFirst,0,gearRatio.Length-1)];
	}
	public float GetGearOut(){
		return topGear*GetGear();
	}
	public float GetClutch(){
		return (isNeitrale)?0:(1f-clutch);
	}
	public float GetSide(){
		return Mathf.Sign(GetGear());
	}
}
