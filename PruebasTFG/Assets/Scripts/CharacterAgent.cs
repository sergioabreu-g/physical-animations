using System;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class CharacterAgent : Agent {
    [Header("--- GENERAL ---")]
    [Tooltip("Remember to also configure the Fixed Time Step in the project config.")]
    [SerializeField] private int _solverIterations = 13;
    [SerializeField] private int _velSolverIterations = 13;
    [SerializeField] private float _maxAngularVelocity = 50;
    [SerializeField] private float _targetVel = 1;
    [SerializeField] private bool _dynamicAnimation = false;
    [SerializeField] private bool _setAnimatedToPhysicalPos = false;

    private Vector3 _animatedAnimatorStartPos;


    [Header("--- Reward ---")]
    [SerializeField] private float _maxCoMDistance = 0.5f;
    [SerializeField] private float _maxRootAngle = 70;

    [SerializeField]
    private float _rotationRewardConstant = -2,
                    _angularVelRewardConstant = -0.1f,
                    _endEffectorRewardConstant = -40,
                    _centerOfMassConstant = -10,
                    _targetVelConstant = -2.5f;

    [SerializeField] private float _rotationsRewardWeight = 0.65f,
                                    _angularVelsRewardWeight = 0.1f,
                                    _endEffectorRewardWeight = 0.15f,
                                    _centerOfMassRewardWeight = 0.1f,
                                    _targetVelRewardWeight = 0;

    private Vector3 _CoM, _refCoM;

    private BehaviorParameters _behaviorParameters;

    [Header("--- BODY ---")]
    [SerializeField] private Rigidbody _physicalRoot;
    [SerializeField] private Rigidbody _animatedRoot;
    
    [Tooltip("IMPORTANT: The first BodyPart must be the root of the body.")]
    [SerializeField] private BodyPart[] _bodyParts;

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
                _animatedRoot = _animatedAnimator.GetBoneTransform(HumanBodyBones.Hips).GetComponent<Rigidbody>();
            if (_physicalRoot == null)
                _physicalRoot = _physicalAnimator.GetBoneTransform(HumanBodyBones.Hips).GetComponent<Rigidbody>();
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
        _animatedAnimatorStartPos = _animatedAnimator.transform.position;

        foreach (BodyPart bodyPart in _bodyParts) {
            bodyPart.rb.solverIterations = _solverIterations;
            bodyPart.rb.solverVelocityIterations = _velSolverIterations;
            bodyPart.rb.maxAngularVelocity = _maxAngularVelocity;
        }
    }

    private void FixedUpdate() {
        if (_setAnimatedToPhysicalPos) {
            var animatedFixedPos = _physicalRoot.position;
            animatedFixedPos.y = _animatedRoot.position.y;
            animatedFixedPos.x = _animatedRoot.position.x;

            _animatedAnimator.transform.position =
                animatedFixedPos - _animatedRoot.transform.localPosition;
        }

        if (CheckEndConditions())
            EndEpisode();
    }

    private void UpdateReward() {
        float totalReward = CalculateTotalReward();
        AddReward(totalReward);

        //Debug.Log("Total (fixed) reward: " + (totalReward * Time.fixedDeltaTime));
    }
    
    public float CalculateTotalReward()
    {
        float rotationsReward = 0;
        float angularVelsReward = 0;
        float endEffectorReward = 0;

        Quaternion worldToRootSpace = Quaternion.Inverse(_physicalRoot.rotation);
        Quaternion worldToRefRootSpace = Quaternion.Inverse(_animatedRoot.rotation);

        foreach (BodyPart bp in _bodyParts) {
            rotationsReward += PartialRotationReward(bp);
            angularVelsReward += PartialAngularVelReward(bp);
            if (bp.endEffector)
                endEffectorReward += PartialEndEffectorReward(bp);
        }

        rotationsReward = Mathf.Exp(_rotationRewardConstant * rotationsReward);
        angularVelsReward = Mathf.Exp(_angularVelRewardConstant * angularVelsReward);
        endEffectorReward = Mathf.Exp(_endEffectorRewardConstant * endEffectorReward);
        float centerOfMassReward = CalculateCenterOfMassReward();
        float targetVelReward = CalculateTargetVelReward();

        float totalReward = _rotationsRewardWeight * rotationsReward
                            + _angularVelsRewardWeight * angularVelsReward
                            + _endEffectorRewardWeight * endEffectorReward
                            + _centerOfMassRewardWeight * centerOfMassReward
                            + _targetVelRewardWeight * targetVelReward;

        /*
        Debug.Log("rotation: " + rotationsReward);
        Debug.Log("vels: " + angularVelsReward);
        Debug.Log("com: " + centerOfMassReward);
        */

        return totalReward;
    }

    private float PartialRotationReward(in BodyPart bp) {
        return Mathf.Pow(Quaternion.Angle(bp.rb.rotation, bp.animatedEquivalent.rotation) * Mathf.Deg2Rad, 2);
    }

    private float PartialAngularVelReward(in BodyPart bp)
    {
        return Mathf.Pow(Vector3.Distance(bp.rb.angularVelocity, bp.animatedEquivalent.angularVelocity), 2);
    }

    private float PartialEndEffectorReward(in BodyPart bp)
    {
        return Mathf.Pow(Vector3.Distance(bp.rb.position, bp.animatedEquivalent.position), 2);
    }

    private float CalculateCenterOfMassReward()
    {
        float totalMass = 0;
        _CoM = Vector3.zero;
        _refCoM = Vector3.zero;

        foreach (BodyPart bp in _bodyParts)
        {
            _CoM += bp.rb.position * bp.rb.mass;
            _refCoM += bp.animatedEquivalent.position * bp.rb.mass;

            totalMass += bp.rb.mass;
        }

        _CoM /= totalMass;
        _refCoM /= totalMass;

        float CoMReward = Mathf.Exp(_centerOfMassConstant * Mathf.Pow(Vector3.Distance(_CoM, _refCoM), 2));

        return CoMReward;
    }

    public float CalculateTargetVelReward()
    {
        return Mathf.Exp(_targetVelConstant * Mathf.Pow(Mathf.Max(0, _targetVel - _physicalRoot.velocity.z), 2));
    }

    // Returns whether the episode must end or not given the current character state
    private bool CheckEndConditions()
    {
        bool CoMDistance = Vector3.Distance(_CoM, _refCoM) > _maxCoMDistance;
        bool rootAngle = Quaternion.Angle(_animatedRoot.rotation, _physicalRoot.rotation) > _maxRootAngle;

        bool groundContact = false;
        foreach (BodyPart bp in _bodyParts)
        {
            if (!bp.canTouchGround && bp.touchingGround)
            {
                groundContact = true;
                break;
            }
        }
        
        return CoMDistance || rootAngle || groundContact;
    }

    //Preparacion de un nuevo intento
    public override void OnEpisodeBegin() {
        _animatedAnimator.transform.position = _animatedAnimatorStartPos;

        foreach (BodyPart bodypart in _bodyParts)
            bodypart.ResetPose();
    }

    void DefineObservationActionSpaces() {
        // Observation space
        int physicalRotations = _bodyParts.Length * 4;
        int physicalVels = _bodyParts.Length * 3;
        int physicalAngularVels = _bodyParts.Length * 3;
        int physicalPositions = _bodyParts.Length * 3;

        _behaviorParameters.BrainParameters.VectorObservationSize =
            physicalRotations + physicalVels + physicalAngularVels + physicalPositions;

        if (_dynamicAnimation)
            _behaviorParameters.BrainParameters.VectorObservationSize++;

        // Action Space
        int targetRotations = 0;
        foreach (BodyPart bodypart in _bodyParts) {
            if (bodypart.joint != null) {
                if (bodypart.joint.angularXMotion != ConfigurableJointMotion.Locked) targetRotations++;
                if (bodypart.joint.angularYMotion != ConfigurableJointMotion.Locked) targetRotations++;
                if (bodypart.joint.angularZMotion != ConfigurableJointMotion.Locked) targetRotations++;
            }
        }
        _behaviorParameters.BrainParameters.VectorActionSize[0] = targetRotations;
    }

    //Recoleccion de informacion necesaria para tomar decisiones
    public override void CollectObservations(VectorSensor sensor) {
        // ROOT observations are different from the rest of the BodyParts
        sensor.AddObservation(_physicalRoot.rotation);
        sensor.AddObservation(_physicalRoot.velocity);
        sensor.AddObservation(_physicalRoot.angularVelocity);
        sensor.AddObservation(_animatedRoot.position - _physicalRoot.position);

        // BODY PARTS
        Quaternion worldToRootSpace = Quaternion.Inverse(_physicalRoot.rotation);
        for (int i = 1; i < _bodyParts.Length; i++) {
            sensor.AddObservation(worldToRootSpace * _bodyParts[i].rb.rotation);
            sensor.AddObservation(worldToRootSpace * _bodyParts[i].rb.velocity);
            sensor.AddObservation(worldToRootSpace * _bodyParts[i].rb.angularVelocity);
            sensor.AddObservation(_bodyParts[i].rb.position - _physicalRoot.position);
        }

        if (_dynamicAnimation)
        {
            float animationPercent = _animatedAnimator.GetCurrentAnimatorStateInfo(0).normalizedTime;
            animationPercent = animationPercent - Mathf.Floor(animationPercent);
            sensor.AddObservation(animationPercent);
            //Debug.Log(animationPercent);
        }
    }

    //Ejecuta las acciones y determina las recompensas. Recibe un vector con la información necesaria para llevar a cabo las acciones
    public override void OnActionReceived(float[] vectorAction) {
        UpdateReward();

        int v = 0;
        foreach (BodyPart bodypart in _bodyParts) {
            if (bodypart.joint == null) continue;

            var rotX = bodypart.joint.angularXMotion == ConfigurableJointMotion.Locked? 0 : vectorAction[v++];
            var rotY = bodypart.joint.angularYMotion == ConfigurableJointMotion.Locked? 0 : vectorAction[v++];
            var rotZ = bodypart.joint.angularZMotion == ConfigurableJointMotion.Locked? 0 : vectorAction[v++];

            bodypart.SetTargetRotation(rotX, rotY, rotZ);
        }
    }
}