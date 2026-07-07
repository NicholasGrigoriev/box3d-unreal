#include "Box3DTypes.h"

#include "Box3DBodyComponent.h"
#include "box3d/box3d.h"

namespace Box3D
{
	UBox3DBodyComponent* ResolveComponent(b3ShapeId ShapeId)
	{
		if (!b3Shape_IsValid(ShapeId))
		{
			return nullptr;
		}

		const b3BodyId BodyId = b3Shape_GetBody(ShapeId);
		if (!b3Body_IsValid(BodyId))
		{
			return nullptr;
		}

		// Bodies created by UBox3DBodyComponent always carry the component as
		// userData; bodies created directly through the C API (e.g. box3d.Smoke)
		// carry null or something else and resolve to nullptr here.
		UObject* UserData = static_cast<UObject*>(b3Body_GetUserData(BodyId));
		return Cast<UBox3DBodyComponent>(UserData);
	}

	int32 AllocateSelfCollisionGroup()
	{
		static int32 Counter = 0;
		return --Counter;
	}
}
