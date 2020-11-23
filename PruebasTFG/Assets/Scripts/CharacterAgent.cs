using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Policies;
using System;

public class CharacterAgent : Agent {
    [Header("--- GENERAL ---")]
    [SerializeField] private int _solverIterations = 13;
    [SerializeField] private int _velSolverIterations = 13;
    [SerializeField] private float _maxAngularVelocity = 50;
    [SerializeField] private float _maxInclination = 50;
    [SerializeField] private bool _feedWorldRotations = false;
    [SerializeField] private float _negativePercentage = 0.25f;

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

    [Header("--- POSITION TARGETS ---")]
    [SerializeField] private TargetPair[] _targets;

    [Serializable]
    public struct TargetPair {
        public Transform bodyPart;
        public Transform target;
        public float maxDistance;
        public float maxAngle;
        public float positionRandomRange;
        public float rotationRandomRange;

        private Vector3 _initialPos;
        private Quaternion _initialRot;

        public void Init() {
            _initialPos = target.position;
            _initialRot = target.rotation;
        }

        public void Randomize() {
            target.position = _initialPos + new Vector3(UnityEngine.Random.Range(-positionRandomRange, positionRandomRange),
                                                        UnityEngine.Random.Range(-positionRandomRange, positionRandomRange),
                                                        UnityEngine.Random.Range(-positionRandomRange, positionRandomRange));

            target.rotation = _initialRot * Quaternion.Euler(new Vector3(UnityEngine.Random.Range(-rotationRandomRange, rotationRandomRange),
                                                        UnityEngine.Random.Range(-rotationRandomRange, rotationRandomRange),
                                                        UnityEngine.Random.Range(-rotationRandomRange, rotationRandomRange)));
        }
    }


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

        for (int i = 0; i < _targets.Length; i++)
            _targets[i].Init();

        SupportBasis = new List<BodyPart>();
    }

    private void FixedUpdate() {
        CalculateBalance();
        SyncAnimatedBody();
        CalculateReward();


        // TEMPORARILY DISABLED
        // UpdateJointTargets();
    }

    private void CalculateReward() {
        //float inclinationReward = 1 - (Mathf.Log(_balanceAngle) / Mathf.Log(_maxInclination));+
        //float inclinationReward = Mathf.Pow(1 - (_balanceAngle / _maxInclination), 2);
        float targetReward = TargetsReward();

        float totalReward = targetReward;
        totalReward = (totalReward * (1 + _negativePercentage)) - _negativePercentage;

        AddReward(totalReward);

        // If some body part is in contact with the floor but it shouldn't be, give no reward in this step
        foreach (BodyPart bodyPart in _bodyParts)
            if (bodyPart.touchingGround && !bodyPart.canTouchGround) {
                SetReward(0);
                break;
            }
    }

    private float TargetsReward() {
        float targetReward = 0;
        foreach (TargetPair targetPair in _targets) {
            float dist = Vector3.Distance(targetPair.bodyPart.position, targetPair.target.position);
            float angle = Quaternion.Angle(targetPair.bodyPart.rotation, targetPair.target.rotation);

            if (dist > targetPair.maxDistance || angle > targetPair.maxAngle)
                EndEpisode();

            float distReward = Mathf.Pow(1 - (dist / targetPair.maxDistance), 2);
            float angleReward = Mathf.Pow(1 - (angle / targetPair.maxAngle), 2);

            targetReward += distReward * angleReward;
        }
        targetReward = targetReward / _targets.Length;

        return targetReward;
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
            _balanceAngle = _maxInclination;
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

        foreach (TargetPair targetPair in _targets)
            targetPair.Randomize();
    }

    void DefineObservationActionSpaces() {
        // Observation Space
        int physicalBoneRotations = _bodyParts.Length * 4;
        int relativeBonePos = _bodyParts.Length * 3;
        int angularVels = _bodyParts.Length * 3;
        int vels = _bodyParts.Length * 3;
        int touchingGrounds = _bodyParts.Length;
        int relativeStrengths = (_bodyParts.Length - 1);

        int targetRelPositions = _targets.Length * 3;
        int targetRelRotations = _targets.Length * 4;

        int balanceVector = 3;
        int uprightVector = 3; // upright vector normalized (-gravity, generally Vector.up)

        _behaviorParameters.BrainParameters.VectorObservationSize =
            physicalBoneRotations + relativeBonePos + balanceVector + angularVels + vels
            + relativeStrengths + uprightVector + touchingGrounds + targetRelPositions
            + targetRelRotations;

        // Action Space
        int targetRotations = 0;
        foreach (BodyPart bodypart in _bodyParts) {
            if (bodypart.joint != null) {
                if (bodypart.joint.angularXMotion != ConfigurableJointMotion.Locked) targetRotations++;
                if (bodypart.joint.angularYMotion != ConfigurableJointMotion.Locked) targetRotations++;
                if (bodypart.joint.angularZMotion != ConfigurableJointMotion.Locked) targetRotations++;
            }
        }
        _behaviorParameters.BrainParameters.VectorActionSize[0] = targetRotations + relativeStrengths;
    }

    //Recoleccion de informacion necesaria para tomar decisiones
    public override void CollectObservations(VectorSensor sensor) {
        foreach (BodyPart bodypart in _bodyParts) {
            sensor.AddObservation(_feedWorldRotations ? bodypart.rb.rotation : bodypart.rb.transform.localRotation);
            sensor.AddObservation(_physicalRoot.InverseTransformPoint(bodypart.rb.position));
            sensor.AddObservation(_physicalRoot.InverseTransformDirection(bodypart.rb.angularVelocity));
            sensor.AddObservation(_physicalRoot.InverseTransformDirection(bodypart.rb.velocity));
            sensor.AddObservation(bodypart.touchingGround);

            if (bodypart.joint != null)
                sensor.AddObservation(bodypart.RelativeStrength);
        }

        foreach (TargetPair targetPair in _targets) {
            sensor.AddObservation(_physicalRoot.InverseTransformDirection(targetPair.target.position - targetPair.bodyPart.position));
            sensor.AddObservation(Quaternion.FromToRotation(targetPair.bodyPart.forward, targetPair.target.forward));
        }

        sensor.AddObservation(_physicalRoot.InverseTransformDirection(_balanceVector.normalized));
        sensor.AddObservation(_physicalRoot.InverseTransformDirection(-Physics.gravity.normalized));
    }

    //Ejecuta las acciones y determina las recompensas. Recibe un vector con la información necesaria para llevar a cabo las acciones
    public override void OnActionReceived(float[] vectorAction) {
        int v = 0;
        foreach (BodyPart bodypart in _bodyParts) {
            if (bodypart.joint == null) continue;

            var rotX = bodypart.joint.angularXMotion == ConfigurableJointMotion.Locked? 0 : vectorAction[v++];
            var rotY = bodypart.joint.angularYMotion == ConfigurableJointMotion.Locked? 0 : vectorAction[v++];
            var rotZ = bodypart.joint.angularZMotion == ConfigurableJointMotion.Locked? 0 : vectorAction[v++];

            bodypart.SetJointTargetRotation(rotX, rotY, rotZ);
            bodypart.RelativeStrength = (vectorAction[v++] + 1) / 2;
        }
        
        if (_balanceAngle > _maxInclination)
            EndEpisode();
    }
}