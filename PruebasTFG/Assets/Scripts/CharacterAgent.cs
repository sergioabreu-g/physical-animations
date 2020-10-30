using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using UnityEngine.XR.WSA.Input;
using Unity.MLAgents.Policies;

public class CharacterAgent : Agent {
    [Header("--- GENERAL ---")]
    [SerializeField] private int _solverIterations = 13;
    [SerializeField] private int _velSolverIterations = 13;
    [SerializeField] private float _maxAngularVelocity = 50;
    [SerializeField] private bool _feedAbsoluteRotations = false;

    private BehaviorParameters _behaviorParameters;

    [Header("--- BODY ---")]
    [SerializeField] private Transform _physicalRoot;
    [SerializeField] private Transform _animatedRoot;
    [SerializeField] private Transform[] _supportBasis;
    private Vector3 _CoM, _CoS;  // Center of Mass & Center of Support
    private Vector3 _balanceVector;
    float _balanceAngle;
    float _lastBalanceAngle;

    [SerializeField] private Transform[] _animatedBones;
    [SerializeField] private ConfigurableJoint[] _joints;
    [SerializeField] private Rigidbody[] _rigidbodies;
    private Quaternion[] _initialJointsRotation;
    private Vector3[] _initialJointsPosition;
    private Vector3 _initialPosition;

    [Header("--- ANIMATORS ---")]
    [SerializeField] private Animator _animatedAnimator;
    [SerializeField] private Animator _physicalAnimator;


    private void OnValidate() {
        // Automatically retrieve the necessary references
        var animators = GetComponentsInChildren<Animator>();
        if (animators.Length >= 2) {
            if (_animatedAnimator == null) _animatedAnimator = animators[0];
            if (_physicalAnimator == null) _physicalAnimator = animators[1];

            if (_animatedRoot == null)
                _animatedRoot = _animatedAnimator.GetBoneTransform(HumanBodyBones.Hips);
            if (_physicalRoot == null)
                _physicalRoot = _physicalAnimator.GetBoneTransform(HumanBodyBones.Hips);

            if (_supportBasis.Length == 0) {
                _supportBasis = new Transform[2];
                _supportBasis[0] = _physicalAnimator.GetBoneTransform(HumanBodyBones.LeftFoot);
                _supportBasis[1] = _physicalAnimator.GetBoneTransform(HumanBodyBones.RightFoot);
            }
        }

        if (_animatedBones.Length == 0) _animatedBones = _animatedRoot?.GetComponentsInChildren<Transform>();
        if (_joints.Length == 0) _joints = _physicalRoot?.GetComponentsInChildren<ConfigurableJoint>();
        if (_rigidbodies.Length == 0) _rigidbodies = _physicalRoot?.GetComponentsInChildren<Rigidbody>();

        if (_animatedBones.Length != _joints.Length)
            Debug.LogError("Animated Bones, Joints and Rigidbodies (arrays) should all have the same length and be equivalent to each other.");

        _behaviorParameters = GetComponent<BehaviorParameters>();
        DefineObservationActionSpaces();
    }

    void DefineObservationActionSpaces() {
        int physicalBoneRotations = _rigidbodies.Length * 4;
        int targetRots = _rigidbodies.Length * 4;
        int balanceVector = 3;
        int angularVels = _rigidbodies.Length * 3;
        int vels = _rigidbodies.Length * 3;
        int supportBasis = _supportBasis.Length * 3;
        int CoM = 3;
        int CoS = 3;

        // ENOUGH WITH BALANCE VECTOR? int uprightVector = 3; // upright vector normalized (-gravity, generally Vector.up)
        int balanceAngle = 1;

        _behaviorParameters.BrainParameters.VectorObservationSize = physicalBoneRotations + balanceVector + angularVels + vels + targetRots + balanceAngle + supportBasis + CoM + CoS;

        int targetRotations = physicalBoneRotations;
        _behaviorParameters.BrainParameters.VectorActionSize[0] = targetRotations;
    }

    void Start() {
        foreach (Rigidbody rb in _rigidbodies) {
            rb.solverIterations = _solverIterations;
            rb.solverVelocityIterations = _velSolverIterations;
            rb.maxAngularVelocity = _maxAngularVelocity;
        }

        _initialJointsRotation = new Quaternion[_joints.Length];
        _initialJointsPosition = new Vector3[_joints.Length];
        for (int i = 0; i < _joints.Length; i++) {
            _initialJointsRotation[i] = _joints[i].transform.localRotation;
            _initialJointsPosition[i] = _joints[i].transform.position;
        }

        _initialPosition = _physicalRoot.position;
    }

    private void FixedUpdate() {
        CalculateBalance();
        SyncAnimatedBody();

        AddReward(_lastBalanceAngle - _balanceAngle);
        _lastBalanceAngle = _balanceAngle;

        // TEMPORARILY DISABLED
        // UpdateJointTargets();
    }

    private void CalculateBalance() {
        // Calculate the center of mass of the body
        _CoM = Vector3.zero;
        float c = 0f;

        foreach (Rigidbody rb in _rigidbodies) {
            _CoM += rb.worldCenterOfMass * rb.mass;
            c += rb.mass;
        }
        _CoM /= c;

        _CoS = Vector3.zero;
        // Calculate the center of support (generally the middle point of both feet)
        foreach (Transform support in _supportBasis)
            _CoS += support.position;
        _CoS /= _supportBasis.Length;

        // Calculate the vector balance (the vector that goes from the CoS to the CoM)
        _balanceVector = _CoM - _CoS;

        // Calculate the balance angle (the angle deviation between the balance vector and the upright vector)
        _balanceAngle = Vector3.Angle(_balanceVector, -Physics.gravity.normalized);
    }

    /// <summary> Makes the physical bones match the rotation of the animated ones </summary>
    private void UpdateJointTargets() {
        for (int i = 0; i < _joints.Length; i++) {
            ConfigurableJointExtensions.SetTargetRotationLocal(_joints[i], _animatedBones[i + 1].localRotation, _initialJointsRotation[i]);
        }
    }

    /// <summary> Updates the rotation and position of the animated body's root
    /// to match the ones of the physical.</summary>
    private void SyncAnimatedBody() {
        _animatedAnimator.transform.position = _physicalRoot.position
                        + (_animatedAnimator.transform.position - _animatedRoot.position);
        _animatedAnimator.transform.rotation = _physicalRoot.rotation;
    }

    //Preparacion de un nuevo intento
    public override void OnEpisodeBegin() {
        _physicalRoot.position = _initialPosition;

        for (int i = 0; i < _rigidbodies.Length; i++) {
            _rigidbodies[i].angularVelocity = Vector3.zero;
            _rigidbodies[i].velocity = Vector3.zero;
            _rigidbodies[i].transform.localRotation = _initialJointsRotation[i];
            _rigidbodies[i].transform.position = _initialJointsPosition[i];
        }
    }


    //Recoleccion de informacion necesaria para tomar decisiones
    public override void CollectObservations(VectorSensor sensor) {
        foreach (Rigidbody rb in _rigidbodies) {
            sensor.AddObservation(_feedAbsoluteRotations ? rb.rotation : rb.transform.localRotation);
            sensor.AddObservation(rb.angularVelocity);
            sensor.AddObservation(rb.velocity);
        }
        foreach (ConfigurableJoint joint in _joints) {
            sensor.AddObservation(joint.targetRotation);
        }
        foreach(Transform t in _supportBasis) {
            sensor.AddObservation(t.position);
        }

        sensor.AddObservation(_balanceVector);
        sensor.AddObservation(_balanceAngle);
        sensor.AddObservation(_CoM);
        sensor.AddObservation(_CoS);
    }

    //Ejecuta las acciones y determina las recomensas. Recibe un vector con la información necesaria para llevar a cabo las acciones
    public override void OnActionReceived(float[] vectorAction) {
        for (int i = 0; i < _joints.Length; i += 4) {
            Quaternion q = new Quaternion(vectorAction[i], vectorAction[i + 1], vectorAction[i + 2], vectorAction[i + 3]);
            _joints[i].targetRotation = q;
        }

        if (_balanceAngle > 40) {
            AddReward(-10.0f);
            EndEpisode();
        }
    }
}