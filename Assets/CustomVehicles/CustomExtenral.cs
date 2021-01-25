using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class CustomExtenral
{
	public static float GetFactorBounciness(this PhysicMaterial aM1, PhysicMaterial aM2)
	{
		if (aM1.bounceCombine == PhysicMaterialCombine.Maximum || aM2.bounceCombine == PhysicMaterialCombine.Maximum)
			return Mathf.Max(aM1.bounciness, aM2.bounciness);
		if (aM1.bounceCombine == PhysicMaterialCombine.Multiply || aM2.bounceCombine == PhysicMaterialCombine.Multiply)
			return aM1.bounciness * aM2.bounciness;
		if (aM1.bounceCombine == PhysicMaterialCombine.Minimum || aM2.bounceCombine == PhysicMaterialCombine.Minimum)
			return Mathf.Min(aM1.bounciness, aM2.bounciness);
		return (aM1.bounciness + aM2.bounciness)*0.5f;
	}
	
	public static float GetFactorDynamic(this PhysicMaterial aM1, PhysicMaterial aM2)
	{
		if (aM1.frictionCombine == PhysicMaterialCombine.Maximum || aM2.frictionCombine == PhysicMaterialCombine.Maximum)
			return Mathf.Max(aM1.dynamicFriction, aM2.dynamicFriction);
		if (aM1.frictionCombine == PhysicMaterialCombine.Multiply || aM2.frictionCombine == PhysicMaterialCombine.Multiply)
			return aM1.dynamicFriction* aM2.dynamicFriction;
		if (aM1.frictionCombine == PhysicMaterialCombine.Minimum || aM2.frictionCombine == PhysicMaterialCombine.Minimum)
			return Mathf.Min(aM1.dynamicFriction, aM2.dynamicFriction);
		return (aM1.dynamicFriction+ aM2.dynamicFriction)*0.5f;
	}
	public static float GetFactorStatic(this PhysicMaterial aM1, PhysicMaterial aM2)
	{
		if (aM1.frictionCombine == PhysicMaterialCombine.Maximum || aM2.frictionCombine == PhysicMaterialCombine.Maximum)
			return Mathf.Max(aM1.staticFriction, aM2.staticFriction);
		if (aM1.frictionCombine == PhysicMaterialCombine.Multiply || aM2.frictionCombine == PhysicMaterialCombine.Multiply)
			return aM1.staticFriction* aM2.staticFriction;
		if (aM1.frictionCombine == PhysicMaterialCombine.Minimum || aM2.frictionCombine == PhysicMaterialCombine.Minimum)
			return Mathf.Min(aM1.staticFriction, aM2.staticFriction);
		return (aM1.staticFriction+ aM2.staticFriction)*0.5f;
	}
	public static float GetFactorDynamicStatic(this PhysicMaterial aM1, PhysicMaterial aM2)
	{
		if (aM1.frictionCombine == PhysicMaterialCombine.Maximum || aM2.frictionCombine == PhysicMaterialCombine.Maximum)
			return Mathf.Max(aM1.dynamicFriction, aM2.staticFriction);
		if (aM1.frictionCombine == PhysicMaterialCombine.Multiply || aM2.frictionCombine == PhysicMaterialCombine.Multiply)
			return aM1.dynamicFriction* aM2.staticFriction;
		if (aM1.frictionCombine == PhysicMaterialCombine.Minimum || aM2.frictionCombine == PhysicMaterialCombine.Minimum)
			return Mathf.Min(aM1.dynamicFriction, aM2.staticFriction);
		return (aM1.dynamicFriction+ aM2.staticFriction)*0.5f;
	}
}
