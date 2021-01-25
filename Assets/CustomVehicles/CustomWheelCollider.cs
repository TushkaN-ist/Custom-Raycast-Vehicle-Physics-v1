using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CustomWheelCollider : MonoBehaviour
{
	static Dictionary<string,Mesh> meshes=new Dictionary<string, Mesh>();
	Rigidbody mainBody;
	[SerializePrivateVariables]
	MeshCollider collider;
	[SerializePrivateVariables]
	Rigidbody body;
	[Header("Graphic")]
	public Transform graphicObj;
	[Header("Suspension")]
	public float restLength=.5f;
	public float springTravel=.5f;
	[Range(-1,1)]
	public int side=0;
	[SerializeField,Range(0,1)]
	private float _springTravelOffset=.5f;
	public float springStiggness=30000;
	public float damperStiggness=4000;
	public float springTravelOffset{
		get {
			return _springTravelOffset;
		}
		set {
			_springTravelOffset = value;
			minLength = restLength - springTravel*springTravelOffset;
			maxLength = restLength + springTravel*(1f-springTravelOffset);
		}
	}
	
	[Header("Wheel")]
	public float mass=15;
	public float radius=0.39f;
	public float width = 0.2f;
	[Range(-1,1)]
	public float center=0;
	
	[Range(4,32)]
	public byte details=8;
	[Range(0,5)]
	public byte detailsWidth=0;
	[Range(0,0.5f)]
	public float round=0;
	
	[SerializeField]
	float _RPM;
	
	public float spdKMH{get; protected set;}
	
	protected float minLength;
	protected float maxLength;
	protected float lastLength;
	protected float springLength;
	protected float springVelocity;
	protected float springforce;
	protected float damperforce;
	
	protected Vector3 suspensionForce;
	public Vector3 wheelLocalVelocity{get{return _wheelLocalVelocity;}}
	[SerializeField]
	protected Vector3 _wheelLocalVelocity;
	
	
	protected Mesh wheelMesh;
	
	public float wheelRot;
	
	public float angularVelocity;
	
	// Pacejka coefficients
	public float[] a={1.0f,-60f,1688f,4140f,6.026f,0f,-0.3589f,1f,0f,-6.111f/1000f,-3.244f/100f,0f,0f,0f,0f};
	public float[] b={1.0f,-60f,1588f,0f,229f,0f,0f,0f,-10f,0f,0f};
	
	float maxSlip;
	float maxAngle;
	Vector3 groundNormal;
	
	// Wheel angular inertia in kg * m^2
	public float inertia = 2.2f;
	// inputs
	// engine torque applied to this wheel
	public float driveTorque = 0;
	// engine braking and other drivetrain friction torques applied to this wheel
	public float driveFrictionTorque = 0;
	// brake input
	public float brake = 0;
	// handbrake input
	public float handbrake = 0;
	// drivetrain inertia as currently connected to this wheel
	public float drivetrainInertia = 0;
	// suspension force externally applied (by anti-roll bars)
	public float suspensionForceInput = 0;
	
	// Maximal braking torque (in Nm)
	public float brakeFrictionTorque = 4000;
	// Maximal handbrake torque (in Nm)
	public float handbrakeFrictionTorque = 0;
	// Base friction torque (in Nm)
	public float frictionTorque = 10;
	
	public float grip = 1.0f;
	
	// This function is called when the script is loaded or a value is changed in the inspector (Called in the editor only).
	protected void OnValidate() {
		inertia = (mass*radius*radius/2f)*2f;
		//inertia*=2;
		mainBody=transform.parent.GetComponentInParent<Rigidbody>();
		radius = Mathf.Max(0,radius);
		wheelMesh=GenerateWheel(radius,width,details,detailsWidth,false);
		if (enabled)
		{
			if (body==null){
				body=new GameObject("body").AddComponent<Rigidbody>();
				body.transform.SetParent(transform,false);
				body.gameObject.hideFlags = HideFlags.HideAndDontSave;
				body.hideFlags = HideFlags.HideAndDontSave | HideFlags.HideInInspector;
				body.detectCollisions=false;
				body.isKinematic=true;
				body.mass = mass;
				collider = body.gameObject.AddComponent<MeshCollider>();
				collider.enabled=true;
				collider.convex=true;
				collider.hideFlags = HideFlags.HideAndDontSave | HideFlags.HideInInspector;
			}
			if (collider!=null && collider.sharedMesh!=null && collider.sharedMesh!=wheelMesh)
				DestroyImmediate(collider.sharedMesh);
		
			collider.sharedMesh = wheelMesh;
		}
		springTravelOffset=_springTravelOffset;
		if (!Application.isPlaying)
			springLength=maxLength;
			
		if (graphicObj!=null)
			GetGraphic(graphicObj);
	}
	
    // Start is called before the first frame update
    void Start()
    {
	    OnValidate();
	    InitSlipMaxima();
    }
	
	public float SetRPMTorque(float targetRPM,float gear,float effective,float clutch,float engineEnertia){
		float baseTorque = (targetRPM - _RPM) * clutch / (inertia + engineEnertia);
		driveTorque = baseTorque * engineEnertia * gear*effective;
		//Debug.Log(transform.name+":"+Mathf.Clamp(wheelLocalVelocity.y/damperStiggness,0,10f));
		return baseTorque*inertia*effective;
	}
	
    // Update is called once per frame
    void Update()
	{
		//wheelRot=(wheelRot+(angularVelocity*Time.deltaTime * Mathf.Rad2Deg))%360f;
		wheelRot=(wheelRot+(angularVelocity*Time.deltaTime * Mathf.Rad2Deg))%360f;
		if (graphicObj!=null)
			GetGraphic(graphicObj);
	}
    
	RaycastHit hit;
	public bool isGround{get;private set;}
		
	public void GetGraphic(out Vector3 pos,out Quaternion rot){
		pos = transform.position-transform.up*springLength;
		rot = transform.rotation*Quaternion.Euler(wheelRot,0,0);
	}
	public void GetGraphic(Transform wheelObj){
		Vector3 pos;
		Quaternion rot;
		GetGraphic(out pos,out rot);
		wheelObj.position = pos;
		wheelObj.rotation = rot;
	}
	
	// This function is called every fixed framerate frame, if the MonoBehaviour is enabled.
	protected void FixedUpdate()
	{
		UpdateCustomPhysic();
	}
	
	void UpdateCustomPhysic(){
		
		Vector3 pos = transform.position;
		
		float dist=maxLength;
		if ((isGround = body.SweepTest(-transform.up,out hit,maxLength,QueryTriggerInteraction.Ignore))){
			dist = hit.distance;
		}
		
		lastLength = springLength;
		springLength = Mathf.Clamp(dist,minLength,maxLength);
		springVelocity = Mathf.Max((lastLength-springLength) / Time.fixedDeltaTime,0);
		springforce = springStiggness * (restLength-springLength);
		damperforce = damperStiggness * springVelocity;
		
		_wheelLocalVelocity.y = springforce+damperforce;
		
		suspensionForce = (_wheelLocalVelocity.y)*(transform.up+hit.normal)/2f;
		
		float totalInertia = inertia + drivetrainInertia;
		float driveAngularDelta = driveTorque * Time.fixedDeltaTime / totalInertia;
		float totalFrictionTorque = brakeFrictionTorque * brake + handbrakeFrictionTorque * handbrake + frictionTorque + driveFrictionTorque;
		float frictionAngularDelta = totalFrictionTorque * Time.fixedDeltaTime / totalInertia;
		float oldAngle=0;
		if (isGround)
		{
			groundNormal = transform.InverseTransformDirection (hit.normal);
			Vector3 wheelVelo=mainBody.GetPointVelocity(hit.point);
			if (hit.rigidbody!=null){
				wheelVelo-=hit.rigidbody.GetPointVelocity(hit.point);
			}
			Vector3 localVelo = transform.InverseTransformDirection (wheelVelo);
			Vector3 roadForce = RoadForce(_wheelLocalVelocity.y,grip,0,ref oldAngle,totalInertia,driveAngularDelta,frictionAngularDelta,wheelVelo,ref angularVelocity,localVelo,groundNormal);
			_wheelLocalVelocity.x =Mathf.Lerp(localVelo.x,roadForce.x*Time.fixedDeltaTime*radius,Mathf.Abs(localVelo.x*0.75f));
			_wheelLocalVelocity.z =roadForce.z*Time.fixedDeltaTime*radius;
			roadForce = transform.TransformDirection (roadForce);
			mainBody.AddForceAtPosition(suspensionForce,transform.position);
			mainBody.AddForceAtPosition(roadForce,transform.position-transform.up*(springLength+radius*center));
			if (hit.rigidbody!=null){
				hit.rigidbody.AddForceAtPosition(-roadForce-suspensionForce,transform.position-transform.up*(springLength+radius*center));
			}
		}
		else
		{
			_wheelLocalVelocity.x=0;
			_wheelLocalVelocity.z=0;
			angularVelocity += driveAngularDelta;
			if (Mathf.Abs(angularVelocity) > frictionAngularDelta)
				angularVelocity -= frictionAngularDelta * Mathf.Sign(angularVelocity);
			else
				angularVelocity = 0;
		}
		_RPM = ((angularVelocity*Time.fixedDeltaTime * Mathf.Rad2Deg)/360f)*(60*60);
		spdKMH = _RPM*radius*2*100*0.001885f;
	}
	
	#if UNITY_EDITOR
	// Implement OnDrawGizmos if you want to draw gizmos that are also pickable and always drawn.
	protected void OnDrawGizmos() {
		Gizmos.DrawWireSphere(transform.position-transform.up*(springLength+radius*center),0.1f);
		Gizmos.color = Color.green;
		Gizmos.DrawWireMesh(collider.sharedMesh,transform.position-transform.up*springLength,transform.rotation,transform.lossyScale);

		if (isGround){
			Vector3 pos = hit.point;
			Gizmos.color = Color.green;
			Gizmos.DrawRay(pos,suspensionForce/damperStiggness);
			Gizmos.color = Color.blue;
			Gizmos.DrawRay(pos,-transform.forward*wheelLocalVelocity.z);
			Gizmos.color = Color.red;
			Gizmos.DrawRay(pos,-transform.right*wheelLocalVelocity.x);
		}
		if ((!UnityEditor.EditorApplication.isPlaying || UnityEditor.EditorApplication.isPlaying && UnityEditor.EditorApplication.isPaused) && graphicObj!=null)
			GetGraphic(graphicObj);
		
	}
	#endif
	
	Mesh GenerateWheel(float radius,float width,int detailWheel=11,int detal=0,bool cache=true){
		Mesh wheel=null;
		string name = radius+":"+width+":"+detailWheel;
		if (!meshes.TryGetValue(name,out wheel)){
			wheel = new Mesh();
			wheel.name = name;
			List<Vector3> vertex=new List<Vector3>();
			List<int> triangles = new List<int>();
			float angle=-90;
			detal++;
			
			float rad = radius;
			for (int d = 0; d <= detal; d++) {
				rad = radius*((1f-round)+(Mathf.Abs(Mathf.Sin(d/(float)detal*Mathf.PI))*round));
				for (int i = 0; i < detailWheel; i++) {
					vertex.Add(new Vector3(-width/2f+(width*(d/(float)detal)),Mathf.Sin(angle*Mathf.Deg2Rad)*rad,Mathf.Cos(angle*Mathf.Deg2Rad)*rad));
					angle+=360f/detailWheel;
				}
			}
			int div=detailWheel-1;
			int point = 0;
			for (int i = point; i < point+div; i++) {
				triangles.Add(point);
				triangles.Add(i);
				triangles.Add(i+1);
			}
			point = vertex.Count-detailWheel;
			for (int i = point; i < point+div; i++) {
				triangles.Add(point);
				triangles.Add(i+1);
				triangles.Add(i);
			}
			int p=0;
			for (int d = 0; d < detal; d++) {
				p=d*detailWheel;
				for (int i = 0; i < detailWheel; i++) {
					triangles.Add(p+(i%detailWheel));
					triangles.Add(p+(i+detailWheel));
					triangles.Add(p+(i+1)%detailWheel);
					
					triangles.Add(p+((i+1)%detailWheel+detailWheel));
					triangles.Add(p+(i+1)%detailWheel);
					triangles.Add(p+(i+detailWheel));
				}
			}
			wheel.SetVertices(vertex);
			wheel.SetTriangles(triangles,0);
			wheel.RecalculateNormals();
			
			if (cache)
				meshes.Add(name,wheel);
		}
		return wheel;
	}
	
	/////////////////
	
	
	float CalcLongitudinalForce(float Fz,float slip)
	{
		Fz*=0.001f;//convert to kN
		slip*=100f; //covert to %
		float uP=b[1]*Fz+b[2];
		float D=uP*Fz;	
		float B=((b[3]*Fz+b[4])*Mathf.Exp(-b[5]*Fz))/(b[0]*uP);
		float S=slip+b[9]*Fz+b[10];
		float E=b[6]*Fz*Fz+b[7]*Fz+b[8];
		float Fx=D*Mathf.Sin(b[0]*Mathf.Atan(S*B+E*(Mathf.Atan(S*B)-S*B)));
		return Fx;
	}
	
	float CalcLateralForce(float Fz,float slipAngle)
	{
		Fz*=0.001f;//convert to kN
		slipAngle*=(360f/(2*Mathf.PI)); //convert angle to deg
		float uP=a[1]*Fz+a[2];
		float D=uP*Fz;
		float B=(a[3]*Mathf.Sin(2*Mathf.Atan(Fz/a[4])))/(a[0]*uP*Fz);
		float S=slipAngle+a[9]*Fz+a[10];
		float E=a[6]*Fz+a[7];
		float Sv=a[12]*Fz+a[13];
		float Fy=D*Mathf.Sin(a[0]*Mathf.Atan(S*B+E*(Mathf.Atan(S*B)-S*B)))+Sv;
		return Fy;
	}
	
	float CalcLongitudinalForceUnit(float Fz,float slip)
	{
		return CalcLongitudinalForce(Fz,slip*maxSlip);
	}
	
	float CalcLateralForceUnit(float Fz,float slipAngle)
	{
		return CalcLongitudinalForce(Fz,slipAngle*maxAngle);
	}

	Vector3 CombinedForce(float Fz,float slip,float slipAngle,Vector3 wheelLocalVelocity,Vector3 groundNormal)
	{
		float unitSlip = slip/maxSlip;
		float unitAngle = slipAngle/maxAngle;
		float p = Mathf.Sqrt(unitSlip*unitSlip + unitAngle*unitAngle);
		if(p > Mathf.Epsilon)
		{
			/*if (slip < -0.8f)
				return -wheelLocalVelocity.normalized * (Mathf.Abs(unitAngle/p * CalcLateralForceUnit(Fz,p)) + Mathf.Abs(unitSlip/p * CalcLongitudinalForceUnit(Fz,p)));
			else
			{*/
				Vector3 forward = new Vector3( 0, -groundNormal.z, groundNormal.y);
				return Vector3.right * unitAngle/p * CalcLateralForceUnit(Fz,p) + forward * unitSlip/p * CalcLongitudinalForceUnit(Fz,p);
			//}
		}
		else
			return Vector3.zero;
	}

	void InitSlipMaxima()
	{
		const float stepSize = 0.001f;
		const float testNormalForce = 4000f;
		float force = 0;
		for (float slip = stepSize;;slip += stepSize)
		{
			float newForce = CalcLongitudinalForce(testNormalForce,slip);
			if (force<newForce)
				force = newForce;
			else {
				maxSlip = slip-stepSize;
				break;
			}
		}
		force = 0;
		for (float slipAngle = stepSize;;slipAngle += stepSize)
		{
			float newForce = CalcLateralForce(testNormalForce,slipAngle);
			if (force<newForce)
				force = newForce;
			else {
				maxAngle = slipAngle-stepSize;
				break;
			}
		}
	}
	
	float SlipRatio (Vector3 worldVelocity,float angularVelocity)
	{
		const float fullSlipVelo = 4.0f;

		float wheelRoadVelo = Vector3.Dot (transform.forward,worldVelocity);
		if (wheelRoadVelo == 0)
			return 0;
		
		float absRoadVelo = Mathf.Abs (wheelRoadVelo);
		float damping = Mathf.Clamp01( absRoadVelo / fullSlipVelo );
		
		float wheelTireVelo = angularVelocity *radius;
		return (wheelTireVelo - wheelRoadVelo) / absRoadVelo * damping;
	}

	float SlipAngle (Vector3 wheelLocalVelocity)
	{
		const float fullAngleVelo = 2.0f;
		
		Vector3 wheelMotionDirection = wheelLocalVelocity;
		wheelMotionDirection.y = 0;

		if (wheelMotionDirection.sqrMagnitude < Mathf.Epsilon)
			return 0;
				
		float sinSlipAngle = wheelMotionDirection.normalized.x;
		sinSlipAngle = Mathf.Clamp(sinSlipAngle, -1, 1); // To avoid precision errors.

		float damping = Mathf.Clamp01( wheelLocalVelocity.magnitude / fullAngleVelo );
		
		return -Mathf.Asin(sinSlipAngle) * damping * damping;
	}
	Vector3 RoadForce (float normalForce,float grip,float newAngle,ref float oldAngle,float totalInertia,float driveAngularDelta,float frictionAngularDelta,Vector3 worldVelocity,ref float angularVelocity,Vector3 wheelLocalVelocity,Vector3 normalGroundLocal) {
		int slipRes=(int)((100.0f-Mathf.Abs(angularVelocity))/(10.0f));
		if (slipRes < 1)
			slipRes = 1;
		float invSlipRes = (1.0f/(float)slipRes);
		Vector3 totalForce = Vector3.zero,forward = Vector3.zero,right = Vector3.zero;
		Quaternion localRotation,inverseLocalRotation;
		for (int i=0; i<slipRes; i++)
		{
			float f = i * 1.0f/(float)slipRes;
			localRotation = Quaternion.Euler (0, oldAngle + (newAngle - oldAngle) * f, 0);
			
			Vector3 force = invSlipRes * grip * CombinedForce (normalForce, SlipRatio (worldVelocity,angularVelocity), SlipAngle (wheelLocalVelocity),wheelLocalVelocity,normalGroundLocal);
			Vector3 worldForce = localRotation * force;
			angularVelocity -= (force.z * radius * Time.deltaTime) / totalInertia;
			angularVelocity += driveAngularDelta;
			if (Mathf.Abs(angularVelocity) > frictionAngularDelta)
				angularVelocity -= frictionAngularDelta * Mathf.Sign(angularVelocity);
			else
				angularVelocity = 0;
				
			totalForce += worldForce;
		}
		
		oldAngle = newAngle;
		return totalForce;
	}
}
