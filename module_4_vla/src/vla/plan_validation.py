"""
Plan validation and safety checks for VLA systems
"""

import json
import re
from typing import Dict, List, Any, Tuple, Optional
from dataclasses import dataclass
from enum import Enum


class SafetyLevel(Enum):
    """Safety levels for plan validation"""
    CRITICAL = "critical"
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"
    NONE = "none"


class ValidationIssueType(Enum):
    """Types of validation issues"""
    SAFETY_HAZARD = "safety_hazard"
    PHYSICAL_IMPOSSIBILITY = "physical_impossibility"
    RESOURCE_UNAVAILABILITY = "resource_unavailability"
    LOGICAL_INCONSISTENCY = "logical_inconsistency"
    ENVIRONMENTAL_CONSTRAINT = "environmental_constraint"
    HUMAN_SAFETY = "human_safety"


@dataclass
class ValidationIssue:
    """Represents a validation issue found in a plan"""
    issue_type: ValidationIssueType
    severity: SafetyLevel
    description: str
    affected_actions: List[int]  # Indices of affected actions
    suggested_fix: str
    confidence: float  # How confident we are in this issue


@data class ValidationResult:
    """Result of plan validation"""
    is_valid: bool
    issues: List[ValidationIssue]
    confidence: float
    safety_score: float  # 0.0 to 1.0, higher is safer
    risk_assessment: Dict[str, Any]


class PlanValidator:
    """
    Validates robot plans for safety and feasibility
    """

    def __init__(self):
        # Robot capabilities and constraints
        self.robot_specs = {
            "max_payload": 3.0,  # kg
            "max_reach": 1.5,    # meters
            "max_speed": 1.0,    # m/s
            "navigation_speed": 0.5,  # m/s
            "manipulation_precision": 0.01,  # meters
            "safe_distance_to_human": 0.5,  # meters
            "max_operation_time": 3600  # seconds
        }

        # Known dangerous actions
        self.dangerous_actions = [
            "move_at_high_speed",
            "grasp_heavy_object",
            "navigate_crowded_area",
            "manipulate_sharp_object",
            "operate_in_tight_space"
        ]

        # Safety rules
        self.safety_rules = [
            self._check_human_proximity,
            self._check_payload_limits,
            self._check_reach_constraints,
            self._check_navigation_safety,
            self._check_manipulation_safety,
            self._check_environmental_constraints,
            self._check_logical_consistency
        ]

    def validate_plan(self, action_sequence: List[Dict[str, Any]], context: Dict[str, Any] = None) -> ValidationResult:
        """
        Validate a plan for safety and feasibility

        Args:
            action_sequence: List of actions to validate
            context: Environmental and contextual information

        Returns:
            ValidationResult with validation results
        """
        context = context or {}
        issues = []

        # Run all safety checks
        for safety_check in self.safety_rules:
            check_issues = safety_check(action_sequence, context)
            issues.extend(check_issues)

        # Calculate overall safety score
        safety_score = self._calculate_safety_score(issues, action_sequence)

        # Determine if plan is valid
        critical_issues = [issue for issue in issues if issue.severity == SafetyLevel.CRITICAL]
        is_valid = len(critical_issues) == 0

        # Calculate confidence based on issue severity and count
        confidence = self._calculate_validation_confidence(issues, action_sequence)

        # Risk assessment
        risk_assessment = self._generate_risk_assessment(issues, action_sequence)

        return ValidationResult(
            is_valid=is_valid,
            issues=issues,
            confidence=confidence,
            safety_score=safety_score,
            risk_assessment=risk_assessment
        )

    def _check_human_proximity(self, action_sequence: List[Dict[str, Any]], context: Dict[str, Any]) -> List[ValidationIssue]:
        """Check for actions that may violate human safety"""
        issues = []

        for i, action in enumerate(action_sequence):
            if action.get('action') in ['navigate_to_location', 'move_forward', 'approach_object']:
                # Check if action might bring robot too close to humans
                if context.get('humans_nearby', False):
                    human_distance = context.get('closest_human_distance', float('inf'))
                    if human_distance < self.robot_specs['safe_distance_to_human']:
                        issues.append(ValidationIssue(
                            issue_type=ValidationIssueType.HUMAN_SAFETY,
                            severity=SafetyLevel.HIGH,
                            description=f"Action {action['action']} may bring robot too close to human (current distance: {human_distance}m)",
                            affected_actions=[i],
                            suggested_fix=f"Add safety check for human proximity before executing {action['action']}",
                            confidence=0.9
                        ))

        return issues

    def _check_payload_limits(self, action_sequence: List[Dict[str, Any]], context: Dict[str, Any]) -> List[ValidationIssue]:
        """Check for actions that exceed payload limits"""
        issues = []

        for i, action in enumerate(action_sequence):
            if action.get('action') == 'grasp_object':
                object_weight = action.get('parameters', {}).get('weight', 0.0)
                if object_weight > self.robot_specs['max_payload']:
                    issues.append(ValidationIssue(
                        issue_type=ValidationIssueType.PHYSICAL_IMPOSSIBILITY,
                        severity=SafetyLevel.HIGH,
                        description=f"Object weighs {object_weight}kg, exceeding robot payload limit of {self.robot_specs['max_payload']}kg",
                        affected_actions=[i],
                        suggested_fix="Select a lighter object or use specialized equipment",
                        confidence=1.0
                    ))

        return issues

    def _check_reach_constraints(self, action_sequence: List[Dict[str, Any]], context: Dict[str, Any]) -> List[ValidationIssue]:
        """Check for actions that exceed reach constraints"""
        issues = []

        for i, action in enumerate(action_sequence):
            if action.get('action') in ['grasp_object', 'manipulate_object']:
                distance = action.get('parameters', {}).get('distance', 0.0)
                if distance > self.robot_specs['max_reach']:
                    issues.append(ValidationIssue(
                        issue_type=ValidationIssueType.PHYSICAL_IMPOSSIBILITY,
                        severity=SafetyLevel.HIGH,
                        description=f"Target object at {distance}m is beyond robot reach limit of {self.robot_specs['max_reach']}m",
                        affected_actions=[i],
                        suggested_fix="Navigate closer to the object before attempting grasp",
                        confidence=1.0
                    ))

        return issues

    def _check_navigation_safety(self, action_sequence: List[Dict[str, Any]], context: Dict[str, Any]) -> List[ValidationIssue]:
        """Check for navigation-related safety issues"""
        issues = []

        for i, action in enumerate(action_sequence):
            if action.get('action') == 'navigate_to_location':
                # Check for speed-related issues
                speed = action.get('parameters', {}).get('speed', self.robot_specs['navigation_speed'])
                if speed > self.robot_specs['max_speed']:
                    issues.append(ValidationIssue(
                        issue_type=ValidationIssueType.SAFETY_HAZARD,
                        severity=SafetyLevel.MEDIUM,
                        description=f"Navigation speed {speed}m/s exceeds safe limit of {self.robot_specs['max_speed']}m/s",
                        affected_actions=[i],
                        suggested_fix=f"Reduce navigation speed to maximum of {self.robot_specs['max_speed']}m/s",
                        confidence=0.8
                    ))

                # Check for crowded areas
                if context.get('area_crowded', False):
                    issues.append(ValidationIssue(
                        issue_type=ValidationIssueType.HUMAN_SAFETY,
                        severity=SafetyLevel.HIGH,
                        description="Navigation in crowded area detected",
                        affected_actions=[i],
                        suggested_fix="Use pedestrian-aware navigation mode or wait for area to clear",
                        confidence=0.9
                    ))

        return issues

    def _check_manipulation_safety(self, action_sequence: List[Dict[str, Any]], context: Dict[str, Any]) -> List[ValidationIssue]:
        """Check for manipulation-related safety issues"""
        issues = []

        for i, action in enumerate(action_sequence):
            if action.get('action') in ['grasp_object', 'manipulate_object']:
                obj_type = action.get('parameters', {}).get('object_type', '').lower()

                # Check for fragile objects
                if any(fragile in obj_type for fragile in ['glass', 'ceramic', 'porcelain', 'fragile']):
                    issues.append(ValidationIssue(
                        issue_type=ValidationIssueType.SAFETY_HAZARD,
                        severity=SafetyLevel.MEDIUM,
                        description=f"Manipulating fragile object '{obj_type}' requires extra care",
                        affected_actions=[i],
                        suggested_fix="Use gentle grasp mode and slow transport",
                        confidence=0.7
                    ))

                # Check for sharp objects
                if any(sharp in obj_type for sharp in ['knife', 'blade', 'sharp', 'cutting']):
                    issues.append(ValidationIssue(
                        issue_type=ValidationIssueType.SAFETY_HAZARD,
                        severity=SafetyLevel.HIGH,
                        description=f"Manipulating sharp object '{obj_type}' detected",
                        affected_actions=[i],
                        suggested_fix="Use protective grasp mode and ensure safe orientation",
                        confidence=0.8
                    ))

        return issues

    def _check_environmental_constraints(self, action_sequence: List[Dict[str, Any]], context: Dict[str, Any]) -> List[ValidationIssue]:
        """Check for environmental constraint violations"""
        issues = []

        # Check for environmental constraints in context
        forbidden_areas = context.get('forbidden_areas', [])
        restricted_objects = context.get('restricted_objects', [])

        for i, action in enumerate(action_sequence):
            if action.get('action') == 'navigate_to_location':
                target_location = action.get('parameters', {}).get('location', '').lower()
                if any(forbidden in target_location for forbidden in forbidden_areas):
                    issues.append(ValidationIssue(
                        issue_type=ValidationIssueType.ENVIRONMENTAL_CONSTRAINT,
                        severity=SafetyLevel.HIGH,
                        description=f"Navigation to forbidden area '{target_location}'",
                        affected_actions=[i],
                        suggested_fix=f"Select alternative destination outside of {', '.join(forbidden_areas)}",
                        confidence=1.0
                    ))

            elif action.get('action') == 'grasp_object':
                object_name = action.get('parameters', {}).get('object_name', '').lower()
                if any(restricted in object_name for restricted in restricted_objects):
                    issues.append(ValidationIssue(
                        issue_type=ValidationIssueType.ENVIRONMENTAL_CONSTRAINT,
                        severity=SafetyLevel.HIGH,
                        description=f"Attempting to grasp restricted object '{object_name}'",
                        affected_actions=[i],
                        suggested_fix=f"Avoid grasping objects in {', '.join(restricted_objects)}",
                        confidence=1.0
                    ))

        return issues

    def _check_logical_consistency(self, action_sequence: List[Dict[str, Any]], context: Dict[str, Any]) -> List[ValidationIssue]:
        """Check for logical inconsistencies in the plan"""
        issues = []

        # Check for impossible action sequences
        for i in range(len(action_sequence) - 1):
            current_action = action_sequence[i].get('action')
            next_action = action_sequence[i + 1].get('action')

            # Check for impossible transitions
            if current_action == 'grasp_object' and next_action == 'grasp_object':
                issues.append(ValidationIssue(
                    issue_type=ValidationIssueType.LOGICAL_INCONSISTENCY,
                    severity=SafetyLevel.MEDIUM,
                    description="Attempting to grasp a second object without releasing the first",
                    affected_actions=[i, i + 1],
                    suggested_fix="Release current object before grasping a new one",
                    confidence=0.9
                ))

        return issues

    def _calculate_safety_score(self, issues: List[ValidationIssue], action_sequence: List[Dict[str, Any]]) -> float:
        """Calculate overall safety score based on issues"""
        if not action_sequence:
            return 1.0  # Empty plan is safe by default

        # Start with perfect safety
        safety_score = 1.0

        # Deduct points for each issue based on severity
        severity_weights = {
            SafetyLevel.CRITICAL: 0.8,
            SafetyLevel.HIGH: 0.5,
            SafetyLevel.MEDIUM: 0.2,
            SafetyLevel.LOW: 0.1,
            SafetyLevel.NONE: 0.0
        }

        for issue in issues:
            safety_score -= severity_weights.get(issue.severity, 0.1)

        # Ensure score stays within bounds
        return max(0.0, min(1.0, safety_score))

    def _calculate_validation_confidence(self, issues: List[ValidationIssue], action_sequence: List[Dict[str, Any]]) -> float:
        """Calculate confidence in the validation result"""
        if not action_sequence:
            return 1.0

        # Base confidence on number and severity of issues
        issue_count = len(issues)
        critical_issues = sum(1 for issue in issues if issue.severity == SafetyLevel.CRITICAL)
        high_issues = sum(1 for issue in issues if issue.severity == SafetyLevel.HIGH)

        # Calculate confidence (lower for more issues)
        base_confidence = 1.0
        issue_penalty = (critical_issues * 0.3) + (high_issues * 0.1) + (issue_count * 0.05)

        confidence = base_confidence - issue_penalty
        return max(0.1, confidence)  # Minimum confidence of 0.1

    def _generate_risk_assessment(self, issues: List[ValidationIssue], action_sequence: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Generate risk assessment for the plan"""
        risk_assessment = {
            "total_actions": len(action_sequence),
            "total_issues": len(issues),
            "critical_issues": len([i for i in issues if i.severity == SafetyLevel.CRITICAL]),
            "high_risk_issues": len([i for i in issues if i.severity == SafetyLevel.HIGH]),
            "medium_risk_issues": len([i for i in issues if i.severity == SafetyLevel.MEDIUM]),
            "low_risk_issues": len([i for i in issues if i.severity == SafetyLevel.LOW]),
            "risk_by_type": {},
            "recommendation": "proceed" if not any(i.severity in [SafetyLevel.CRITICAL, SafetyLevel.HIGH] for i in issues) else "review"
        }

        # Count issues by type
        for issue in issues:
            issue_type = issue.issue_type.value
            risk_assessment["risk_by_type"][issue_type] = risk_assessment["risk_by_type"].get(issue_type, 0) + 1

        return risk_assessment

    def get_safety_report(self, validation_result: ValidationResult) -> str:
        """Generate a human-readable safety report"""
        report = []
        report.append("=== PLAN SAFETY REPORT ===")
        report.append(f"Plan Validity: {'VALID' if validation_result.is_valid else 'INVALID'}")
        report.append(f"Safety Score: {validation_result.safety_score:.2f}/1.0")
        report.append(f"Validation Confidence: {validation_result.confidence:.2f}/1.0")
        report.append("")

        if validation_result.issues:
            report.append("ISSUES FOUND:")
            for issue in validation_result.issues:
                report.append(f"  - {issue.severity.value.upper()}: {issue.description}")
                report.append(f"    Type: {issue.issue_type.value}")
                report.append(f"    Affected Actions: {issue.affected_actions}")
                report.append(f"    Suggestion: {issue.suggested_fix}")
                report.append(f"    Confidence: {issue.confidence:.2f}")
                report.append("")
        else:
            report.append("No safety issues detected!")

        report.append(f"Recommendation: {validation_result.risk_assessment['recommendation'].upper()}")
        report.append("")

        return "\n".join(report)


class SafePlanExecutor:
    """
    Safe execution wrapper that validates plans before execution
    """

    def __init__(self):
        self.validator = PlanValidator()
        self.safety_threshold = 0.7  # Minimum safety score to proceed

    def execute_plan_safely(self, action_sequence: List[Dict[str, Any]], context: Dict[str, Any] = None) -> Tuple[bool, List[ValidationIssue], List[Dict[str, Any]]]:
        """
        Execute a plan safely with validation

        Args:
            action_sequence: List of actions to execute
            context: Environmental context

        Returns:
            Tuple of (can_execute, validation_issues, modified_plan)
        """
        context = context or {}

        # Validate the plan first
        validation_result = self.validator.validate_plan(action_sequence, context)

        # Check if plan meets safety threshold
        if validation_result.safety_score >= self.safety_threshold and validation_result.is_valid:
            return True, validation_result.issues, action_sequence
        else:
            # If plan is unsafe, try to modify it to make it safer
            modified_plan = self._modify_plan_for_safety(action_sequence, validation_result.issues)
            modified_validation = self.validator.validate_plan(modified_plan, context)

            if modified_validation.safety_score >= self.safety_threshold and modified_validation.is_valid:
                return True, modified_validation.issues, modified_plan
            else:
                return False, validation_result.issues, []

    def _modify_plan_for_safety(self, action_sequence: List[Dict[str, Any]], issues: List[ValidationIssue]) -> List[Dict[str, Any]]:
        """
        Modify a plan to address safety issues

        Args:
            action_sequence: Original action sequence
            issues: Validation issues to address

        Returns:
            Modified action sequence with safety improvements
        """
        modified_plan = [action.copy() for action in action_sequence]

        for issue in issues:
            if issue.issue_type == ValidationIssueType.HUMAN_SAFETY:
                # Add safety checks before navigation actions
                for idx in issue.affected_actions:
                    if idx < len(modified_plan):
                        action = modified_plan[idx]
                        if action['action'] in ['navigate_to_location', 'move_forward']:
                            # Add safety parameters
                            if 'parameters' not in action:
                                action['parameters'] = {}
                            action['parameters']['check_human_safety'] = True
                            action['parameters']['safe_distance'] = 0.8  # Increase safe distance

            elif issue.issue_type == ValidationIssueType.PHYSICAL_IMPOSSIBILITY:
                # Modify parameters to make action possible
                for idx in issue.affected_actions:
                    if idx < len(modified_plan):
                        action = modified_plan[idx]
                        if action['action'] == 'navigate_to_location':
                            # Reduce speed for safety
                            if 'parameters' not in action:
                                action['parameters'] = {}
                            action['parameters']['speed'] = min(
                                action['parameters'].get('speed', 0.5),
                                self.validator.robot_specs['navigation_speed']
                            )

        return modified_plan

    def request_human_approval(self, validation_result: ValidationResult, action_sequence: List[Dict[str, Any]]) -> bool:
        """
        Determine if human approval is needed for a plan

        Args:
            validation_result: Validation result for the plan
            action_sequence: Action sequence to evaluate

        Returns:
            True if human approval is recommended
        """
        # Require approval for plans with high or critical issues
        has_critical_or_high_issues = any(
            issue.severity in [SafetyLevel.CRITICAL, SafetyLevel.HIGH]
            for issue in validation_result.issues
        )

        # Also require approval for complex plans with multiple safety concerns
        safety_concerns = sum(
            1 for issue in validation_result.issues
            if issue.severity in [SafetyLevel.MEDIUM, SafetyLevel.HIGH, SafetyLevel.CRITICAL]
        )

        return has_critical_or_high_issues or safety_concerns > 2


# Example usage and testing
def main():
    """
    Example usage of the plan validation system
    """
    print("Testing Plan Validation and Safety Checks...")

    # Initialize validator
    validator = PlanValidator()
    safe_executor = SafePlanExecutor()

    # Test plan with potential safety issues
    test_plan = [
        {
            "action": "navigate_to_location",
            "parameters": {"location": "kitchen", "speed": 1.2},  # Too fast
            "reasoning": "Going to kitchen to get item"
        },
        {
            "action": "grasp_object",
            "parameters": {"object_name": "glass_cup", "weight": 0.8, "distance": 1.8},  # Too far
            "reasoning": "Picking up the cup"
        },
        {
            "action": "transport_object",
            "parameters": {"destination": "living_room"},
            "reasoning": "Moving cup to living room"
        }
    ]

    # Context with safety considerations
    context = {
        "humans_nearby": True,
        "closest_human_distance": 0.3,  # Too close
        "area_crowded": False,
        "forbidden_areas": ["restricted_zone"],
        "restricted_objects": ["dangerous_item"]
    }

    print("Validating plan...")
    result = validator.validate_plan(test_plan, context)

    print(f"Plan is valid: {result.is_valid}")
    print(f"Safety score: {result.safety_score:.2f}")
    print(f"Confidence: {result.confidence:.2f}")
    print(f"Issues found: {len(result.issues)}")

    # Print safety report
    safety_report = validator.get_safety_report(result)
    print("\n" + safety_report)

    # Test safe execution
    print("Testing safe execution...")
    can_execute, issues, modified_plan = safe_executor.execute_plan_safely(test_plan, context)
    print(f"Can execute safely: {can_execute}")
    print(f"Modified plan length: {len(modified_plan)}")

    # Check if human approval is needed
    needs_approval = safe_executor.request_human_approval(result, test_plan)
    print(f"Requires human approval: {needs_approval}")

    print("\nPlan validation test completed!")


if __name__ == "__main__":
    main()