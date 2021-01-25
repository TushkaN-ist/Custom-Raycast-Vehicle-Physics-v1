using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CustomCarController : MonoBehaviour
{
	public Rigidbody body;
	public Transform COM;
	
	public CustomCarEngine engine;
	public CustomCarGearBox gearBox;
	
	public CustomAxe[] axis;
	public float _break,_handBreak;
	public Vector3 localForce;
	public float WheelGearRPM,spdKMH;
	[Range(-1,1)]
	public float wheel=0;
	
	// Start is called before the first frame update
	void Start()
	{
		if (COM && COM.gameObject.active)
			body.centerOfMass = COM.localPosition;
	}
    // Update is called once per frame
	void FixedUpdate()
	{
		//return;
		WheelGearRPM = (engine.rpm/gearBox.GetGearOut());
		float gearOut = Mathf.Abs(gearBox.GetGearOut());
		float backTorque=0;
		float _spdKMH=0;
		foreach (var item in axis)
		{
			item.SetAngle(wheel);
			backTorque+=item.SetRPMTorque(WheelGearRPM,gearOut,gearBox.GetClutch(), engine.inertia);
			item.SetBreaks(_break,_handBreak);
			_spdKMH+=item.GetSpeed();
		}
		engine.wheelTorque=backTorque*gearBox.GetSide();
		spdKMH = Mathf.Lerp(spdKMH,_spdKMH/axis.Length,Time.fixedDeltaTime/.25f);
		localForce = transform.InverseTransformDirection(body.velocity)*60*60/1000f;
	}
	
	public int gear{get;protected set;}
	// Update is called every frame, if the MonoBehaviour is enabled.
	protected void Update()
	{
		//return;
		engine.starter = Input.GetKey(KeyCode.Joystick1Button3);
		engine.throttle=(Input.GetAxis("GamePad AxleR")+1f)/2f;
		wheel=Input.GetAxis("GamePad AxleX");
		_break=0;
		_break=(Input.GetAxis("GamePad AxleL")+1f)/2f;
		_handBreak=0;
		if (Input.GetKey(KeyCode.Joystick1Button0)){
			_handBreak=1;
		}
		if (Input.GetKeyDown(KeyCode.Joystick1Button4)){
			gear=Mathf.Clamp(gear-1,-1,5);
			gearBox.SetGear(gear);
		}
		if (Input.GetKeyDown(KeyCode.Joystick1Button5)){
			gear=Mathf.Clamp(gear+1,-1,5);
			gearBox.SetGear(gear);
		}
		
		if (Input.GetKeyDown(KeyCode.Alpha0)){
			gearBox.SetGear(gear=0);
		}
		if (Input.GetKeyDown(KeyCode.Alpha1)){
			gearBox.SetGear(gear=1);
		}
		if (Input.GetKeyDown(KeyCode.Alpha2)){
			gearBox.SetGear(gear=2);
		}
		if (Input.GetKeyDown(KeyCode.Alpha3)){
			gearBox.SetGear(gear=3);
		}
		if (Input.GetKeyDown(KeyCode.Alpha4)){
			gearBox.SetGear(gear=4);
		}
		if (Input.GetKeyDown(KeyCode.Alpha5)){
			gearBox.SetGear(gear=5);
		}
		if (Input.GetKeyDown(KeyCode.Alpha6)){
			gearBox.SetGear(gear=-1);
		}
	}
}
