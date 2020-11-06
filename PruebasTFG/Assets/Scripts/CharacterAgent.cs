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
    [SerializeField] private bool _feedWorldRotations = false;

    private BehaviorParameters _behaviorParameters;

    [Header("--- BODY ---")]
    [SerializeField] private Transform _physicalRoot;
    [SerializeField] private Transform _animatedRoot;
    private Vector3 _CoM, _CoS;  // Center of Mass & Center of Support
    private Vector3 _balanceVector;
    float _balanceAngle;

    [SerializeField] private BodyPart[] _bodyParts;
    public List<BodyPart> SupportBasis { get; private set; }

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
        }

        if (_bodyParts.Length == 0) SetupBodyParts();

        _behaviorParameters = GetComponent<BehaviorParameters>();
        DefineObservationActionSpaces();
    }

    void SetupBodyParts() {
        Rigidbody[] rigidbodies = _physicalAnimator.GetComponentsInChildren<Rigidbody>();
        _bodyParts = new BodyPart[rigidbodies.Length];

        for (int i = 0; i < rigidbodies.Length; i++) {
            if (!rigidbodies[i].TryGetComponent(out _bodyParts[i]))
                _bodyParts[i] = rigidbodies[i].gameObject.AddComponent<BodyPart>();

            _bodyParts[i].rb = rigidbodies[i];
            _bodyParts[i].joint = rigidbodies[i].gameObject.GetComponent<ConfigurableJoint>();
        }

        Debug.Log("Body parts automatically set up. You need to manually assign the peer animated bones.");
    }

    void Start() {
        foreach (BodyPart bodyPart in _bodyParts) {
            bodyPart.rb.solverIterations = _solverIterations;
            bodyPart.rb.solverVelocityIterations = _velSolverIterations;
            bodyPart.rb.maxAngularVelocity = _maxAngularVelocity;
        }

        SupportBasis = new List<BodyPart>();
    }

    private void FixedUpdate() {
        CalculateBalance();
        SyncAnimatedBody();

        AddReward((40 - _balanceAngle) / 40);

        // TEMPORARILY DISABLED
        // UpdateJointTargets();
    }

    private void CalculateBalance() {
        // Calculate the center of mass of the body
        _CoM = Vector3.zero;
        float c = 0f;

        foreach (BodyPart bodypart in _bodyParts) {
            _CoM += bodypart.rb.worldCenterOfMass * bodypart.rb.mass;
            c += bodypart.rb.mass;
        }
        _CoM /= c;

        _CoS = Vector3.zero;
        SupportBasis.Clear();
        // Calculate the center of support (generally the middle point of both feet)
        foreach (BodyPart bodypart in _bodyParts) {
            if (!bodypart.touchingGround)
                continue;
            SupportBasis.Add(bodypart);
            _CoS += bodypart.rb.position;
        }
        if (SupportBasis.Count != 0) {
            _CoS /= SupportBasis.Count;

            // Calculate the vector balance (the vector that goes from the CoS to the CoM)
            _balanceVector = _CoM - _CoS;

            // Calculate the balance angle (the angle deviation between the balance vector and the upright vector)
            _balanceAngle = Vector3.Angle(_balanceVector, -Physics.gravity.normalized);
        }
        else {
            _balanceAngle = Vector3.Angle(_bodyParts[0].transform.right, -Physics.gravity.normalized);
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
        foreach (BodyPart bodypart in _bodyParts)
            bodypart.Reset();
    }
    void DefineObservationActionSpaces() {
        // Observation Space
        int physicalBoneRotations = _bodyParts.Length * 4;
        int relativeBonePos = _bodyParts.Length * 3;
        int angularVels = _bodyParts.Length * 3;
        int vels = _bodyParts.Length * 3;
        int touchingGrounds = _bodyParts.Length;
        int relativeStrengths = (_bodyParts.Length - 1);

        int balanceVector = 3;
        int uprightVector = 3; // upright vector normalized (-gravity, generally Vector.up)

        _behaviorParameters.BrainParameters.VectorObservationSize =
            physicalBoneRotations + relativeBonePos + balanceVector + angularVels + vels + relativeStrengths + uprightVector + touchingGrounds;

        // Action Space
        int targetRotations = (_bodyParts.Length - 1) * 3;
        _behaviorParameters.BrainParameters.VectorActionSize[0] = targetRotations + relativeStrengths;
    }

    //Recoleccion de informacion necesaria para tomar decisiones
    public override void CollectObservations(VectorSensor sensor) {
        foreach (BodyPart bodypart in _bodyParts) {
            sensor.AddObservation(_feedWorldRotations ? bodypart.rb.rotation : bodypart.rb.transform.localRotation);
            sensor.AddObservation(bodypart.rb.position - _CoM);
            sensor.AddObservation(bodypart.rb.angularVelocity);
            sensor.AddObservation(bodypart.rb.velocity);
            sensor.AddObservation(bodypart.touchingGround);

            if (bodypart.joint != null)
                sensor.AddObservation(bodypart.RelativeStrength);
        }

        sensor.AddObservation(_balanceVector);
        sensor.AddObservation(-Physics.gravity);
    }

    //Ejecuta las acciones y determina las recompensas. Recibe un vector con la información necesaria para llevar a cabo las acciones
    public override void OnActionReceived(float[] vectorAction) {
        int v = 0;

        foreach (BodyPart bodypart in _bodyParts) {
            if (bodypart.joint == null) continue;

            bodypart.SetTargetRotation(vectorAction[v++], vectorAction[v++], vectorAction[v++]);
            bodypart.RelativeStrength = (vectorAction[v++] + 1) / 2;
        }

        if (_balanceAngle > 35) {
            AddReward(-10.0f);
            EndEpisode();
        }
    }
}