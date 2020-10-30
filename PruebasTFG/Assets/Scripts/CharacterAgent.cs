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
    [SerializeField] private Rigidbody _physicalRoot;
    [SerializeField] private Transform _animatedRoot;
    [SerializeField] private Transform[] _supportBasis;
    private Vector3 _CoM, _CoS;  // Center of Mass & Center of Support
    private Vector3 _balanceVector;
    float _balanceAngle;


    private Transform[] AnimatedBones { get; set; }
    private ConfigurableJoint[] Joints { get; set; }
    private Rigidbody[] Rigidbodies { get; set; }
    private Quaternion[] _initialJointsRotation;
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
                _physicalRoot = _physicalAnimator.GetBoneTransform(HumanBodyBones.Hips).GetComponent<Rigidbody>();

            if (_supportBasis.Length == 0) {
                _supportBasis = new Transform[2];
                _supportBasis[0] = _physicalAnimator.GetBoneTransform(HumanBodyBones.LeftFoot);
                _supportBasis[1] = _physicalAnimator.GetBoneTransform(HumanBodyBones.RightFoot);
            }
        }

        AnimatedBones = _animatedRoot?.GetComponentsInChildren<Transform>();
        Joints = _physicalRoot?.GetComponentsInChildren<ConfigurableJoint>();
        Rigidbodies = _physicalRoot?.GetComponentsInChildren<Rigidbody>();

        _behaviorParameters = GetComponent<BehaviorParameters>();
        DefineObservationActionSpaces();
    }

    void DefineObservationActionSpaces() {
        int physicalBoneRotations = Rigidbodies.Length * 4;
        int balanceVector = 3;
        // ENOUGH WITH BALANCE VECTOR? int uprightVector = 3; // upright vector normalized (-gravity, generally Vector.up)
        // ENOUGH WITH BALANCE VECTOR? int balanceAngle = 1;

        _behaviorParameters.BrainParameters.VectorObservationSize = physicalBoneRotations + balanceVector;

        int targetRotations = physicalBoneRotations;
        _behaviorParameters.BrainParameters.VectorActionSize[0] = targetRotations;
    }

    void Start() {
        foreach (Rigidbody rb in Rigidbodies) {
            rb.solverIterations = _solverIterations;
            rb.solverVelocityIterations = _velSolverIterations;
            rb.maxAngularVelocity = _maxAngularVelocity;
        }

        _initialJointsRotation = new Quaternion[Joints.Length];
        for (int i = 0; i < Joints.Length; i++) {
            _initialJointsRotation[i] = Joints[i].transform.localRotation;
        }

        _initialPosition = _physicalRoot.position;
    }

    private void FixedUpdate() {
        CalculateBalance();
        SyncAnimatedBody();

        // TEMPORARILY DISABLED
        // UpdateJointTargets();
    }

    private void CalculateBalance() {
        // Calculate the center of mass of the body
        _CoM = Vector3.zero;
        float c = 0f;

        foreach (Rigidbody rb in Rigidbodies) {
            _CoM += rb.worldCenterOfMass * rb.mass;
            c += rb.mass;
        }
        _CoM /= c;

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
        for (int i = 0; i < Joints.Length; i++) {
            ConfigurableJointExtensions.SetTargetRotationLocal(Joints[i], AnimatedBones[i + 1].localRotation, _initialJointsRotation[i]);
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

        for (int i = 0; i < Rigidbodies.Length; i++) {
            Rigidbodies[i].angularVelocity = Vector3.zero;
            Rigidbodies[i].velocity = Vector3.zero;
            Rigidbodies[i].transform.localRotation = _initialJointsRotation[i];
        }
    }


    //Recoleccion de informacion necesaria para tomar decisiones
    public override void CollectObservations(VectorSensor sensor) {
        foreach (Rigidbody rb in Rigidbodies) {
            sensor.AddObservation(_feedAbsoluteRotations ? rb.rotation : rb.transform.localRotation);
        }
        sensor.AddObservation(_balanceVector);
    }

    //Ejecuta las acciones y determina las recomensas. Recibe un vector con la información necesaria para llevar a cabo las acciones
    public override void OnActionReceived(float[] vectorAction) {
        for (int i = 0; i < Joints.Length; i += 4) {
            Quaternion q = new Quaternion(i, i + 1, i + 2, i + 3);
            Joints[i].targetRotation = q;
        }


        // ESTO SEGURAMENTE NO DEBA IR AQUI NI CALCULARSE ASI, TEMPORAL
        if (_balanceAngle > 60) {
            SetReward(-100.0f);
            EndEpisode();
        }
        else SetReward(1.0f);

    }
}