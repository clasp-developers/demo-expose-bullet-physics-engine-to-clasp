
#include <clasp/clasp.h>
#include <clasp/core/bformat.h>
#include <iostream>
#include <btBulletDynamicsCommon.h>

//
// The demo written in C++
//
void cxx_sphere_drop_simulation(size_t steps, bool suppress_output)
{
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();

    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

    dynamicsWorld->setGravity(btVector3(0, -10, 0));


    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

    btCollisionShape* fallShape = new btSphereShape(1);


    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
    btRigidBody::btRigidBodyConstructionInfo
	groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
    dynamicsWorld->addRigidBody(groundRigidBody);


    btDefaultMotionState* fallMotionState =
	new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 50, 0)));
    btScalar mass = 1;
    btVector3 fallInertia(0, 0, 0);
    fallShape->calculateLocalInertia(mass, fallInertia);
    btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
    btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
    dynamicsWorld->addRigidBody(fallRigidBody);


    for (int i = 0; i < steps; i++) {
	dynamicsWorld->stepSimulation(1 / 60.f, 10);

	btTransform trans;
	fallRigidBody->getMotionState()->getWorldTransform(trans);

	if (!suppress_output) BFORMAT_T(BF("sphere height: %lf\n") % trans.getOrigin().getY());
    }

    dynamicsWorld->removeRigidBody(fallRigidBody);
    delete fallRigidBody->getMotionState();
    delete fallRigidBody;

    dynamicsWorld->removeRigidBody(groundRigidBody);
    delete groundRigidBody->getMotionState();
    delete groundRigidBody;


    delete fallShape;

    delete groundShape;


    delete dynamicsWorld;
    delete solver;
    delete collisionConfiguration;
    delete dispatcher;
    delete broadphase;

}

//
// Translators that translate btScalar to and from Clasp Common Lisp values
//
namespace translate {
    template <>
    struct from_object<btScalar, std::true_type> {
	typedef btScalar DeclareType;
	DeclareType _v;
	from_object(core::T_sp o) : _v(clasp_to_double(gc::As<core::Number_sp>(o))) {};
    };
    template <>
    struct from_object<const btScalar&, std::true_type> {
	typedef btScalar DeclareType;
	DeclareType _v;
	from_object(core::T_sp o) : _v(clasp_to_double(gc::As<core::Number_sp>(o))) {};
    };
    template <>
    struct from_object<const btScalar&, std::false_type> {
	typedef btScalar DeclareType;
	DeclareType _v;
	from_object(core::T_sp o) : _v(clasp_to_double(gc::As<core::Number_sp>(o))) {};
    };
    template <>
    struct to_object<btScalar> {
	typedef btScalar GivenType;
	static core::T_sp convert(GivenType v) {
	    return core::make_single_float(v);
	}
    };
    template <>
    struct to_object<const btScalar&> {
	typedef const btScalar& GivenType;
	static core::T_sp convert(GivenType v) {
	    return core::make_single_float(v);
	}
    };
};


//
// The Foreign class/method/function interface
// This code binds Bullet classes/methods/functions to
//  clasp Common Lisp symbols
//
void startup()
{
    using namespace clbind;
    package("BT") [
		   def("cxx_sphere_drop_simulation",&cxx_sphere_drop_simulation)
		   //		   ,class_<btScalar>("btScalar",no_default_constructor)
		   ,class_<btBroadphaseInterface>("btBroadphaseInterface",no_default_constructor)
		   ,class_<btDbvtBroadphase,btBroadphaseInterface>("btDbvtBroadphase")
		   ,class_<btCollisionConfiguration>("btCollisionConfiguration",no_default_constructor)
		   ,class_<btDefaultCollisionConfiguration,btCollisionConfiguration>("btDefaultCollisionConfiguration")
		   ,class_<btDispatcher>("btDispatcher",no_default_constructor)
		   ,class_<btCollisionDispatcher,btDispatcher>("btCollisionDispatcher",no_default_constructor)
		   .def_constructor("make_btCollisionDispatcher",constructor<btDefaultCollisionConfiguration*>())
		   ,class_<btConstraintSolver>("btConstraintSolver",no_default_constructor)
		   ,class_<btSequentialImpulseConstraintSolver,btConstraintSolver>("btSequentialImpulseConstraintSolver")
		   ,class_<btDiscreteDynamicsWorld>("btDiscreteDynamicsWorld",no_default_constructor)
		   .def_constructor("make_btDiscreteDynamicsWorld",constructor<btDispatcher*, btBroadphaseInterface *, btConstraintSolver *, btCollisionConfiguration *>())
		   .def("stepSimulation",&btDiscreteDynamicsWorld::stepSimulation,policies<>(),
			"(obj time-step max-sub-steps fixed-time-step)")
		   .def("setGravity",&btDiscreteDynamicsWorld::setGravity)
		   .def("addRigidBody",(void(btDiscreteDynamicsWorld::*)(btRigidBody*))&btDiscreteDynamicsWorld::addRigidBody)
		   .def("removeRigidBody",&btDiscreteDynamicsWorld::removeRigidBody)
		   //		       .def("addRigidBody-long",(void(*)(btRigidBody*,short,short))&btDiscreteDynamicsWorld::addRigidBody)
		   ,class_<btVector3>("btVector3",no_default_constructor)
		   .def_constructor("make_btVector3",constructor<btScalar,btScalar,btScalar>())
		   .def("getX",&btVector3::getX)
		   .def("getY",&btVector3::getY)
		   .def("getZ",&btVector3::getZ)
		   ,class_<btCollisionShape>("btCollisionShape",no_default_constructor)
		   ,class_<btConcaveShape,btCollisionShape>("btConcaveShape",no_default_constructor)
		   ,class_<btStaticPlaneShape,btConcaveShape>("btStaticPlaneShape",no_default_constructor)
		   .def_constructor("make_btStaticPlaneShape",constructor<const btVector3&,btScalar>())
		   ,class_<btConvexShape,btCollisionShape>("btConvexShape",no_default_constructor)
		   ,class_<btConvexInternalShape,btConvexShape>("btConvexInteralShape",no_default_constructor)
		   ,class_<btSphereShape,btConvexInternalShape>("btSphereShape",no_default_constructor)
		   .def_constructor("make_btSphereShape",constructor<btScalar>())
		   .def("calculateLocalInertia",&btSphereShape::calculateLocalInertia)
		   ,class_<btTransform>("btTransform")
		   .def_constructor("make_btTransform_quaternion",constructor<const btQuaternion&,const btVector3>())
		   .def("getOrigin",(btVector3&(btTransform::*)())&btTransform::getOrigin)
		   ,def("btTransform_getIdentity",&btTransform::getIdentity)
		   ,class_<btMotionState>("btMotionState",no_default_constructor)
		   ,class_<btDefaultMotionState,btMotionState>("btDefaultMotionState",no_default_constructor)
		   .def_constructor("make_btDefaultMotionState",constructor<const btTransform &, const btTransform &>(),policies<>()
				    ,"(&optional (start-trans (bt:bt-transform-get-identity)) (center-of-mass-offset (bt:bt-transform-get-identity)))")
		   ,class_<btQuaternion>("btQuaternion")
		   .def_constructor("make_btQuaternion",constructor<const btScalar &, const btScalar &, const btScalar &, const btScalar &>())
		   ,class_<btRigidBody::btRigidBodyConstructionInfo>("btRigidBody/btRigidBodyConstructionInfo",no_default_constructor)
		   .def_constructor("make_btRigidBody/btRigidBodyConstructionInfo",constructor<btScalar , btMotionState *, btCollisionShape *, const btVector3 &>(),policies<>()
				    ,"(mass motion-state collision-shape &optional (local-inertia (bt:make-btvector 0.0 0.0 0.0)))")
		   ,class_<btRigidBody>("btRigidBody",no_default_constructor)
		   .def_constructor("make_btRigidBody.btRigidBodyConstructionInfo",constructor<const btRigidBody::btRigidBodyConstructionInfo&>())
		   .def_constructor("make_btRigidBody",constructor<btScalar, btMotionState *, btCollisionShape *, const btVector3 &>(),policies<>(),
				    "(mass motionState collisionShape &optional (localInertia (bt:make-btvector 0 0 0)))")
		   .def("getMotionState",(btMotionState*(btRigidBody::*)())&btRigidBody::getMotionState)
		   ,class_<btMotionState>("btMotionState",no_default_constructor)
		   .def("getWorldTransform",&btMotionState::getWorldTransform)
		   ];
    
}

CLASP_REGISTER_STARTUP(startup);
