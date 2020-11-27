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

    [SerializeField] private float _strengthRewardMultiplier = 2f;
    [SerializeField] private float _touchRewardMultiplier = 0.7f;

    private BehaviorParameters _behaviorParameters;

    [Header("--- BODY ---")]
    [SerializeField] private Transform _physicalRoot;
    [SerializeField] private Transform _animatedRoot;

    private Vector3 _CoM, _CoS;  // Center of Mass & Center of Support
    private Vector3 _balanceVector;
    float _balanceAngle;
    
    [Tooltip("IMPORTANT: The first BodyPart must be the root of the body.")]
    [SerializeField] private BodyPart[] _bodyParts;
    public List<BodyPart> SupportBasis { get; private set; }

    [Header("--- ANIMATORS ---")]
    [SerializeField] private Animator _animatedAnimator;
    [SerializeField] private Animator _physicalAnimator;

    [Header("--- TARGETS ---")]
    [SerializeField] private TargetPair[] _targets;

    [Serializable]
    public struct TargetPair {
        public Transform bodyPart;
        public Transform target;
        public float maxDistance;
        public float maxAngle;
        public bool endEpisode;
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
        _bodyParts = new BodyPart[rigidbodies.Length - 1];

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
        CalculateReward();
    }

    private void CalculateReward() {
        float totalReward = TargetsReward();
        totalReward *= StrengthRewardMultiplier();
        totalReward *= TouchingGroundRewardMultiplier();

        //Debug.Log("Total reward: " + totalReward);
         
        AddReward(totalReward);
    }

    private float TargetsReward() {
        float targetReward = 0;
        foreach (TargetPair targetPair in _targets) {
            float distReward = 1;
            float dist = 0;
            if (targetPair.maxDistance != 0) {
                dist = Vector3.Distance(targetPair.bodyPart.position, targetPair.target.position);
                distReward = 1 - (dist / targetPair.maxDistance);
            }

            float angleReward = 1;
            float angle = 0;
            if (targetPair.maxAngle != 0) {
                angle = Quaternion.Angle(targetPair.bodyPart.rotation, targetPair.target.rotation);
                angleReward = 1 - (angle / targetPair.maxAngle);
            }

            if (targetPair.endEpisode && (dist > targetPair.maxDistance || angle > targetPair.maxAngle))
                EndEpisode();

            targetReward += distReward * angleReward;
        }
        targetReward = targetReward / _targets.Length;

        //Debug.Log("Targets reward: " + targetReward);

        return targetReward;
    }

    private float StrengthRewardMultiplier() {
        float averageStrength = 0;
        foreach (BodyPart bodyPart in _bodyParts)
            averageStrength += bodyPart.RelativeStrength;
        averageStrength /= _bodyParts.Length;
        averageStrength = Mathf.Sqrt(averageStrength);

        //Debug.Log("Strength reward multiplier: " + Mathf.Lerp(1, _strengthRewardMultiplier, 1 - averageStrength));

        return Mathf.Lerp(1, _strengthRewardMultiplier, 1 - averageStrength);
    }

    private float TouchingGroundRewardMultiplier() {
        foreach (BodyPart bodyPart in _bodyParts)
            if (bodyPart.touchingGround && !bodyPart.canTouchGround)
                return _touchRewardMultiplier;

        return 1;
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
    }

    //Preparacion de un nuevo intento
    public override void OnEpisodeBegin() {
        foreach (BodyPart bodypart in _bodyParts)
            bodypart.Reset();
    }

    void DefineObservationActionSpaces() {
        // Observation Space
        int physicalBoneRotations = _bodyParts.Length * 3;
        int relativeBonePos = _bodyParts.Length * 3;
        int angularVels = _bodyParts.Length * 3;
        int vels = _bodyParts.Length * 3;
        int touchingGrounds = _bodyParts.Length;
        int relativeStrengths = (_bodyParts.Length - 1);

        int targetRelPositions = _targets.Length * 3 - 3;
        int targetRelRotations = _targets.Length * 3 - 3;

        int balanceRot = 3;

        _behaviorParameters.BrainParameters.VectorObservationSize =
            physicalBoneRotations + relativeBonePos + balanceRot + angularVels + vels
            + relativeStrengths + touchingGrounds + targetRelPositions
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
        // ROOT observations are different from the rest of the BodyParts
        sensor.AddObservation(_bodyParts[0].rb.angularVelocity);
        sensor.AddObservation(_bodyParts[0].rb.velocity);
        sensor.AddObservation(_bodyParts[0].touchingGround);

        // BODY PARTS
        for (int i = 1; i < _bodyParts.Length; i++) {
            sensor.AddObservation(_physicalRoot.InverseTransformPoint(_bodyParts[i].rb.position));
            sensor.AddObservation(_bodyParts[i].GetJointNormalizedRotation());
            sensor.AddObservation(_physicalRoot.InverseTransformDirection(_bodyParts[i].rb.velocity));
            sensor.AddObservation(_physicalRoot.InverseTransformDirection(_bodyParts[i].rb.angularVelocity));

            sensor.AddObservation(_bodyParts[i].touchingGround);
            sensor.AddObservation(_bodyParts[i].RelativeStrength);
        }

        // TARGETS
        foreach (TargetPair targetPair in _targets) {
            if (targetPair.bodyPart == _physicalRoot) {
                sensor.AddObservation(targetPair.target.position - targetPair.bodyPart.position);
                sensor.AddObservation(Quaternion.FromToRotation(targetPair.bodyPart.forward, targetPair.target.forward).eulerAngles / 360);
                continue;
            }

            sensor.AddObservation(_physicalRoot.InverseTransformDirection(targetPair.target.position - targetPair.bodyPart.position));

            Vector3 bodyForward = _physicalRoot.InverseTransformDirection(targetPair.bodyPart.forward);
            Vector3 targetForward = _physicalRoot.InverseTransformDirection(targetPair.target.forward);

            sensor.AddObservation(Quaternion.FromToRotation(bodyForward, targetForward).eulerAngles / 360);
        }

        // BALANCE
        Vector3 _localBalanceVec = _physicalRoot.InverseTransformDirection(_balanceVector.normalized);
        Vector3 _localGravity = _physicalRoot.InverseTransformDirection(-Physics.gravity.normalized);
        Vector3 _balanceRot = Quaternion.FromToRotation(_localBalanceVec, _localGravity).eulerAngles / 360;
        sensor.AddObservation(_balanceRot);
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
    }
}