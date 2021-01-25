using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CustomAxe : MonoBehaviour
{
	[Range(-90,90)]
	public float wheelAngleIn=0,wheelAngleOut=0;
	[Range(0,1)]
	public float effective=0;
	public CustomWheelCollider[] wheels;
	
	float angleN=0,_break,_handBreak,spdKMH;
	
	public void SetAngle(float normal){
		angleN = normal;
	}
	public void SetBreaks(float _break,float _handBreak){
		SetBreak(_break);
		SetHandBreak(_handBreak);
	}
	public void SetBreak(float _break){
		this._break=_break;
	}
	public void SetHandBreak(float _handBreak){
		this._handBreak=_handBreak;
	}
	public float GetSpeed(){
		return spdKMH;
	}
	public float SetRPMTorque(float targetRPM,float gear,float clutch,float engineEnertia){
		
		float callBack=0;
		foreach (var item in wheels)
		{
			callBack+=item.SetRPMTorque(targetRPM,gear,effective,clutch,engineEnertia);
		}
		return callBack/wheels.Length;
	}
	// This function is called every fixed framerate frame, if the MonoBehaviour is enabled.
	protected void FixedUpdate()
	{
		spdKMH=0;
		int lr=0;
		float angleT=0;
		float angled = wheelAngleIn-wheelAngleOut;
		foreach (var item in wheels)
		{
			angleT = (wheelAngleIn-angled*Mathf.Abs(Mathf.Sign(angleN)-item.side)/2f)*angleN;
			item.transform.localRotation = Quaternion.Euler(0,angleT,0);
			item.brake = _break;
			item.handbrake = _handBreak;
			spdKMH+=item.spdKMH;
			lr++;
		}
		spdKMH/=wheels.Length;
	}
}
