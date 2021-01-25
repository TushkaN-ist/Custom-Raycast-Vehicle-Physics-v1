using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CustomCarEngine : MonoBehaviour
{
	public AnimationCurve torqueCurve,throttleAuto;
	
	[SerializeField]
	public float rpm=0;
	[SerializeField]
	uint rpmCross=4000;
	public float torqueMax=270,backTorque=60,wheelTorque=0;
	
	public float inertia = 0.9f;
	[Range(0,1)]
	public float throttle=0;
	[Range(0.04f,1)]
	public float throttleDelta=0.2f;
	public float starterPower=3300;
	public bool starter=false;
	float _throttle;
	
	#if UNITY_EDITOR
	[Header("Editor")]
	public AnimationCurve powerCurve;
	// This function is called when the script is loaded or a value is changed in the inspector (Called in the editor only).
	protected void OnValidate()
	{
		powerCurve=new AnimationCurve();
		float f=0;
		float t = torqueCurve.keys[torqueCurve.length-1].time;
		while(f<t){
			
			Keyframe kf=new Keyframe();
			kf.time = f;
			kf.inWeight=0;
			kf.outWeight=0;
			kf.tangentMode = 0;
			kf.weightedMode = WeightedMode.Both;
			kf.value=GetPower(f*rpmCross)/torqueMax;
			powerCurve.AddKey(kf);
			f+=0.05f;
		}
	}
	#endif
	
	float GetTorqueRPM(){
		return torqueCurve.Evaluate(rpm/rpmCross)*torqueMax;
	}
	
	float GetTorqueRPM(float gear){
		return torqueCurve.Evaluate(rpm/rpmCross/Mathf.Abs(gear))*torqueMax;
	}
	
	float GetPower(float RPM){
		return (torqueCurve.Evaluate(RPM/rpmCross)*torqueMax)*RPM/rpmCross;
	}
	
	float GetTorque(float throttle){
		return (torqueCurve.Evaluate(rpm/rpmCross)*torqueMax)*throttle;
	}
	float GetAutoThrottle(float throttle){
		return Mathf.Clamp(throttle+throttleAuto.Evaluate(rpm/rpmCross),-1f,1f);
	}
	
	float ProgressRPM(float Throttle,float dtime,float wheelTorque)
	{
		Throttle = GetAutoThrottle(Throttle);
		_throttle = Mathf.Lerp(_throttle,Throttle,dtime/throttleDelta);
		float torq = (GetTorque(_throttle))*dtime-wheelTorque;
		return (torq - Mathf.Pow(1.0f - (_throttle), 2) * (backTorque*dtime));
	}
	
	// Update is called every frame, if the MonoBehaviour is enabled.
	public void UpdateRPM(float dtime)
	{
		if (starter && throttleAuto.Evaluate(rpm/rpmCross)>0)
		{
			rpm+=starterPower*dtime;
		}
		rpm=Mathf.Clamp(rpm+ProgressRPM(throttle,dtime,wheelTorque),0,rpmCross*3);
	}
	// This function is called every fixed framerate frame, if the MonoBehaviour is enabled.
	protected void FixedUpdate()
	{
		UpdateRPM(Time.fixedDeltaTime);
	}
}
