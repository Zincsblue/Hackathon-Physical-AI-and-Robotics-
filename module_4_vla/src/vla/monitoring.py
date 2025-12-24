"""
Monitoring and Logging Module for VLA System

This module provides comprehensive monitoring and logging capabilities for the VLA system,
including performance tracking, error monitoring, and system health checks.
"""

import logging
import time
import json
from datetime import datetime
from typing import Dict, Any, List, Optional
from dataclasses import dataclass
from enum import Enum
import threading
import os
from pathlib import Path


class LogLevel(Enum):
    """Log levels for the VLA system"""
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


@dataclass
class LogEntry:
    """Structure for a log entry"""
    timestamp: datetime
    level: LogLevel
    module: str
    message: str
    details: Optional[Dict[str, Any]] = None


class VLAEventLogger:
    """Centralized event logger for the VLA system"""

    def __init__(self, log_file: Optional[str] = None, console_output: bool = True):
        """
        Initialize the event logger

        Args:
            log_file: Path to log file (optional)
            console_output: Whether to output to console
        """
        self.console_output = console_output
        self.log_entries: List[LogEntry] = []
        self.lock = threading.Lock()

        # Set up file logging if specified
        self.log_file = log_file
        if self.log_file:
            # Create directory if it doesn't exist
            Path(self.log_file).parent.mkdir(parents=True, exist_ok=True)

        # Set up Python logging
        self.logger = logging.getLogger("VLA.Monitoring")
        self.logger.setLevel(logging.DEBUG)

        # Create formatter
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )

        # Add console handler if needed
        if console_output and not self.logger.handlers:
            console_handler = logging.StreamHandler()
            console_handler.setFormatter(formatter)
            self.logger.addHandler(console_handler)

        # Add file handler if specified
        if self.log_file:
            file_handler = logging.FileHandler(self.log_file)
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)

    def log(self, level: LogLevel, module: str, message: str, details: Optional[Dict[str, Any]] = None):
        """Log an event"""
        with self.lock:
            entry = LogEntry(
                timestamp=datetime.now(),
                level=level,
                module=module,
                message=message,
                details=details
            )
            self.log_entries.append(entry)

            # Log to Python logger
            log_method = getattr(self.logger, level.value.lower())
            if details:
                log_method(f"[{module}] {message} | Details: {details}")
            else:
                log_method(f"[{module}] {message}")

    def debug(self, module: str, message: str, details: Optional[Dict[str, Any]] = None):
        """Log a debug message"""
        self.log(LogLevel.DEBUG, module, message, details)

    def info(self, module: str, message: str, details: Optional[Dict[str, Any]] = None):
        """Log an info message"""
        self.log(LogLevel.INFO, module, message, details)

    def warning(self, module: str, message: str, details: Optional[Dict[str, Any]] = None):
        """Log a warning message"""
        self.log(LogLevel.WARNING, module, message, details)

    def error(self, module: str, message: str, details: Optional[Dict[str, Any]] = None):
        """Log an error message"""
        self.log(LogLevel.ERROR, module, message, details)

    def critical(self, module: str, message: str, details: Optional[Dict[str, Any]] = None):
        """Log a critical message"""
        self.log(LogLevel.CRITICAL, module, message, details)

    def get_recent_logs(self, count: int = 10) -> List[LogEntry]:
        """Get recent log entries"""
        with self.lock:
            return self.log_entries[-count:]

    def get_logs_by_level(self, level: LogLevel) -> List[LogEntry]:
        """Get logs filtered by level"""
        with self.lock:
            return [entry for entry in self.log_entries if entry.level == level]

    def get_logs_by_module(self, module: str) -> List[LogEntry]:
        """Get logs filtered by module"""
        with self.lock:
            return [entry for entry in self.log_entries if entry.module == module]

    def clear_logs(self):
        """Clear all logs"""
        with self.lock:
            self.log_entries.clear()


class PerformanceMonitor:
    """Performance monitoring for the VLA system"""

    def __init__(self):
        self.metrics: Dict[str, Any] = {
            "execution_times": [],
            "throughput": 0,
            "error_rates": [],
            "resource_usage": {
                "cpu_percent": [],
                "memory_percent": [],
                "disk_io": []
            },
            "start_time": datetime.now()
        }
        self.lock = threading.Lock()

    def record_execution_time(self, execution_time: float):
        """Record command execution time"""
        with self.lock:
            self.metrics["execution_times"].append(execution_time)
            # Keep only last 1000 entries to prevent memory issues
            if len(self.metrics["execution_times"]) > 1000:
                self.metrics["execution_times"] = self.metrics["execution_times"][-1000:]

    def record_error(self):
        """Record an error occurrence"""
        with self.lock:
            self.metrics["error_rates"].append(datetime.now())
            # Keep only last hour of error data
            one_hour_ago = datetime.now().timestamp() - 3600
            self.metrics["error_rates"] = [
                timestamp for timestamp in self.metrics["error_rates"]
                if timestamp.timestamp() > one_hour_ago
            ]

    def calculate_throughput(self) -> float:
        """Calculate commands per second throughput"""
        with self.lock:
            duration = (datetime.now() - self.metrics["start_time"]).total_seconds()
            if duration > 0:
                return len(self.metrics["execution_times"]) / duration
            return 0.0

    def get_average_execution_time(self) -> float:
        """Get average execution time"""
        with self.lock:
            if self.metrics["execution_times"]:
                return sum(self.metrics["execution_times"]) / len(self.metrics["execution_times"])
            return 0.0

    def get_p95_execution_time(self) -> float:
        """Get 95th percentile execution time"""
        with self.lock:
            if not self.metrics["execution_times"]:
                return 0.0

            sorted_times = sorted(self.metrics["execution_times"])
            index = int(0.95 * len(sorted_times))
            return sorted_times[min(index, len(sorted_times) - 1)]

    def get_error_rate(self) -> float:
        """Get error rate (errors per hour)"""
        with self.lock:
            one_hour_ago = datetime.now().timestamp() - 3600
            recent_errors = [
                timestamp for timestamp in self.metrics["error_rates"]
                if timestamp.timestamp() > one_hour_ago
            ]
            return len(recent_errors)

    def get_performance_summary(self) -> Dict[str, Any]:
        """Get a summary of performance metrics"""
        with self.lock:
            return {
                "total_executions": len(self.metrics["execution_times"]),
                "average_execution_time": self.get_average_execution_time(),
                "p95_execution_time": self.get_p95_execution_time(),
                "throughput_cps": self.calculate_throughput(),
                "error_rate_per_hour": self.get_error_rate(),
                "uptime_seconds": (datetime.now() - self.metrics["start_time"]).total_seconds()
            }


class SystemHealthMonitor:
    """System health monitoring for the VLA system"""

    def __init__(self, event_logger: VLAEventLogger):
        self.event_logger = event_logger
        self.health_status: Dict[str, Any] = {
            "system_status": "healthy",
            "modules": {},
            "last_check": datetime.now(),
            "issues": []
        }
        self.lock = threading.Lock()

    def check_module_health(self, module_name: str, is_healthy: bool, details: Optional[Dict[str, Any]] = None):
        """Check and record the health of a specific module"""
        with self.lock:
            self.health_status["modules"][module_name] = {
                "healthy": is_healthy,
                "last_check": datetime.now(),
                "details": details or {}
            }

            if not is_healthy:
                issue = f"Module {module_name} is unhealthy: {details}"
                self.health_status["issues"].append(issue)
                self.event_logger.warning("HealthMonitor", issue, details)

    def check_system_health(self) -> Dict[str, Any]:
        """Perform a comprehensive system health check"""
        with self.lock:
            # Determine overall system status
            unhealthy_modules = [
                name for name, status in self.health_status["modules"].items()
                if not status["healthy"]
            ]

            if unhealthy_modules:
                self.health_status["system_status"] = "degraded"
            else:
                self.health_status["system_status"] = "healthy"

            self.health_status["last_check"] = datetime.now()

            return self.health_status.copy()

    def get_health_report(self) -> str:
        """Get a formatted health report"""
        health = self.check_system_health()
        report_lines = [
            f"System Health Report - {health['last_check']}",
            f"Overall Status: {health['system_status']}",
            "Module Status:",
        ]

        for module, status in health["modules"].items():
            status_str = "✓ Healthy" if status["healthy"] else "✗ Unhealthy"
            report_lines.append(f"  {module}: {status_str}")

        if health["issues"]:
            report_lines.append("\nIssues:")
            for issue in health["issues"]:
                report_lines.append(f"  - {issue}")

        return "\n".join(report_lines)


class VLAMonitoringSystem:
    """Main monitoring system that combines logging, performance, and health monitoring"""

    def __init__(self, log_file: Optional[str] = None):
        self.event_logger = VLAEventLogger(log_file)
        self.performance_monitor = PerformanceMonitor()
        self.health_monitor = SystemHealthMonitor(self.event_logger)

        # Set up monitoring thread
        self.monitoring_active = False
        self.monitoring_thread = None

    def start_monitoring(self):
        """Start background monitoring"""
        if not self.monitoring_active:
            self.monitoring_active = True
            self.monitoring_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
            self.monitoring_thread.start()

    def stop_monitoring(self):
        """Stop background monitoring"""
        self.monitoring_active = False
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=1.0)

    def _monitoring_loop(self):
        """Background monitoring loop"""
        while self.monitoring_active:
            try:
                # Perform periodic health checks
                self.health_monitor.check_system_health()
                time.sleep(30)  # Check every 30 seconds
            except Exception as e:
                self.event_logger.error("Monitoring", f"Error in monitoring loop: {str(e)}")
                time.sleep(5)  # Wait before retrying

    def log_event(self, level: LogLevel, module: str, message: str, details: Optional[Dict[str, Any]] = None):
        """Log an event"""
        self.event_logger.log(level, module, message, details)

    def record_command_execution(self, execution_time: float, success: bool):
        """Record command execution metrics"""
        self.performance_monitor.record_execution_time(execution_time)
        if not success:
            self.performance_monitor.record_error()

    def check_module_health(self, module_name: str, is_healthy: bool, details: Optional[Dict[str, Any]] = None):
        """Check module health"""
        self.health_monitor.check_module_health(module_name, is_healthy, details)

    def get_system_metrics(self) -> Dict[str, Any]:
        """Get all system metrics"""
        return {
            "performance": self.performance_monitor.get_performance_summary(),
            "health": self.health_monitor.check_system_health(),
            "recent_logs": [vars(log) for log in self.event_logger.get_recent_logs(10)]
        }

    def get_performance_report(self) -> str:
        """Get a formatted performance report"""
        perf_summary = self.performance_monitor.get_performance_summary()
        return (
            f"Performance Report:\n"
            f"  Total Executions: {perf_summary['total_executions']}\n"
            f"  Avg Execution Time: {perf_summary['average_execution_time']:.3f}s\n"
            f"  P95 Execution Time: {perf_summary['p95_execution_time']:.3f}s\n"
            f"  Throughput: {perf_summary['throughput_cps']:.2f} commands/sec\n"
            f"  Error Rate: {perf_summary['error_rate_per_hour']:.2f} errors/hour\n"
            f"  Uptime: {perf_summary['uptime_seconds']:.0f} seconds"
        )


# Global monitoring instance
_monitoring_instance: Optional[VLAMonitoringSystem] = None


def get_monitoring_system() -> VLAMonitoringSystem:
    """Get the global monitoring system instance"""
    global _monitoring_instance
    if _monitoring_instance is None:
        _monitoring_instance = VLAMonitoringSystem("logs/vla_system.log")
    return _monitoring_instance


def initialize_monitoring(log_file: Optional[str] = None) -> VLAMonitoringSystem:
    """Initialize the monitoring system"""
    global _monitoring_instance
    _monitoring_instance = VLAMonitoringSystem(log_file)
    _monitoring_instance.start_monitoring()
    return _monitoring_instance


def shutdown_monitoring():
    """Shutdown the monitoring system"""
    global _monitoring_instance
    if _monitoring_instance:
        _monitoring_instance.stop_monitoring()
        _monitoring_instance = None


# Example usage and testing
if __name__ == "__main__":
    # Initialize monitoring
    monitor = initialize_monitoring("test_vla.log")

    # Log some events
    monitor.log_event(LogLevel.INFO, "TestModule", "Starting test")
    monitor.log_event(LogLevel.WARNING, "TestModule", "This is a warning", {"test": "data"})

    # Record some performance metrics
    import random
    for i in range(100):
        exec_time = random.uniform(0.1, 1.0)
        success = random.choice([True, True, True, True, False])  # 80% success rate
        monitor.record_command_execution(exec_time, success)

    # Check module health
    monitor.check_module_health("LanguageModule", True)
    monitor.check_module_health("VisionModule", False, {"error": "Model not loaded"})

    # Print reports
    print(monitor.get_performance_report())
    print("\n" + "="*50 + "\n")
    print(monitor.health_monitor.get_health_report())

    # Shutdown
    shutdown_monitoring()