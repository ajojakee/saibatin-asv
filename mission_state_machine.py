#!/usr/bin/env python3
"""
Mission State Machine - Dummy implementation for SAIBATIN AZURA
Replace this with actual implementation later
"""

from enum import Enum
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)

class MissionState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    APPROACHING = "approaching"
    CAPTURING = "capturing"
    LOWERING = "lowering"
    UNDERWATER = "underwater"
    RAISING = "raising"
    COMPLETED = "completed"
    ABORTED = "aborted"

@dataclass
class MissionConfig:
    gate_trigger_dist_m: float = 3.0
    speed_threshold_m_s: float = 0.5
    heading_threshold_deg: float = 15.0
    settle_time_s: float = 1.0
    burst_count: int = 3
    burst_interval_s: float = 0.5
    depth_threshold_m: float = 0.5
    lower_timeout_s: float = 20.0
    max_retries: int = 2
    loop_interval_s: float = 0.25

class MissionStateMachine:
    """Dummy state machine for testing. Replace with actual implementation."""
    
    def __init__(self, gates, config, on_capture, on_lower, on_raise, on_navigate, emit_event):
        self.gates = gates
        self.config = config
        self.on_capture = on_capture
        self.on_lower = on_lower
        self.on_raise = on_raise
        self.on_navigate = on_navigate
        self.emit_event = emit_event
        
        self.state = MissionState.IDLE
        self.current_gate_idx = 0
        self.lat = None
        self.lon = None
        self.speed = 0.0
        self.heading = 0.0
        self.running = False
        
        logger.info("Mission State Machine initialized (dummy mode)")
    
    def start(self):
        """Start mission"""
        self.running = True
        self.state = MissionState.NAVIGATING
        self.current_gate_idx = 0
        logger.info(f"Mission started with {len(self.gates)} gates")
        self.emit_event("mission_started", {"gates": len(self.gates)})
    
    def stop(self):
        """Stop mission gracefully"""
        self.running = False
        self.state = MissionState.IDLE
        logger.info("Mission stopped")
        self.emit_event("mission_stopped", {})
    
    def abort(self, reason):
        """Abort mission with reason"""
        self.running = False
        self.state = MissionState.ABORTED
        logger.warning(f"Mission aborted: {reason}")
        self.emit_event("mission_aborted", {"reason": reason})
    
    def update_position(self, lat, lon):
        """Update ASV position from GPS"""
        self.lat = lat
        self.lon = lon
    
    def update_speed(self, speed):
        """Update ASV speed (m/s)"""
        self.speed = speed
    
    def update_heading(self, heading):
        """Update ASV heading (degrees)"""
        self.heading = heading
    
    def get_current_gate(self):
        """Get current gate info"""
        if self.current_gate_idx < len(self.gates):
            return self.gates[self.current_gate_idx]
        return None
    
    def next_gate(self):
        """Move to next gate"""
        self.current_gate_idx += 1
        if self.current_gate_idx >= len(self.gates):
            self.state = MissionState.COMPLETED
            logger.info("Mission completed - all gates passed")
            self.emit_event("mission_completed", {"total_gates": len(self.gates)})
            return False
        return True
