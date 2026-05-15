#!/usr/bin/env python3

"""
player_selection.py

Detects player selections by monitoring block disappearances from their initial 
resting positions (slots). A selection is triggered if a block is missing from 
its slot for a specified duration.
"""

import math
from dataclasses import dataclass
from typing import Dict, Optional

import rospy
from memory_game.msg import BlockArray, PlayerSelection


@dataclass
class SlotState:
    block_id: int
    color: str
    x: float
    y: float
    z: float
    missing_since: Optional[rospy.Time] = None
    selected_latched: bool = False


class PlayerSelectionNode:
    def __init__(self) -> None:
        self.target_frame = rospy.get_param("~target_frame", "panda_link0")
        self.selection_type = rospy.get_param("~selection_type", "slot_disappeared")
        
        # Max allowable drift from initial position
        self.max_slot_distance_m = float(rospy.get_param("~max_slot_distance_m", 0.08))
        
        # Required duration of absence to trigger selection
        self.missing_hold_sec = float(rospy.get_param("~missing_hold_sec", 0.6))
        
        # Minimum time between selections
        self.selection_cooldown_sec = float(rospy.get_param("~selection_cooldown_sec", 0.8))
        
        # Initialization requirements
        self.min_slots_to_initialize = int(rospy.get_param("~min_slots_to_initialize", 3))
        self.baseline_settle_sec = float(rospy.get_param("~baseline_settle_sec", 0.7))

        self.slots_initialized = False
        self.slots: Dict[int, SlotState] = {}
        
        self.baseline_candidate_since = rospy.Time(0)
        self.last_selection_time = rospy.Time(0)
        self.reset_delay_sec = float(rospy.get_param("~reset_delay_sec", 3.0))
        self.all_blocks_missing_since = rospy.Time(0)
        
        self.latest_observed: Dict[int, object] = {}
        self.latest_observed_stamp = rospy.Time(0)

        self.selection_pub = rospy.Publisher("/player_selection", PlayerSelection, queue_size=10)
        rospy.Subscriber("/detected_blocks", BlockArray, self.blocks_callback, queue_size=10)

        rospy.loginfo(
            "player_selection ready: min_slots=%d settle=%.2fs hold=%.2fs",
            self.min_slots_to_initialize,
            self.baseline_settle_sec,
            self.missing_hold_sec
        )

    def blocks_callback(self, msg: BlockArray) -> None:
        received_at = rospy.Time.now()
        stamp = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time.now()
        
        observed = {block.id: block for block in msg.blocks}
        self.latest_observed = observed
        self.latest_observed_stamp = stamp

        if not self.slots_initialized:
            self._handle_initialization(received_at, stamp)
            return

        self.evaluate_slots(observed, stamp)

    def _handle_initialization(self, received_at: rospy.Time, stamp: rospy.Time) -> None:
        # Wait for minimum required blocks
        if len(self.latest_observed) < self.min_slots_to_initialize:
            self.baseline_candidate_since = rospy.Time(0)
            rospy.logwarn_throttle(
                2.0,
                "Waiting for enough blocks to initialize (%d/%d)",
                len(self.latest_observed),
                self.min_slots_to_initialize,
            )
            return

        # Start settle timer
        if self.baseline_candidate_since.is_zero():
            self.baseline_candidate_since = received_at
            rospy.loginfo(
                "Enough blocks detected. Settling for %.2fs...",
                self.baseline_settle_sec,
            )
            return

        # Wait for settle period
        if (received_at - self.baseline_candidate_since).to_sec() < self.baseline_settle_sec:
            time_left = max(0.0, self.baseline_settle_sec - (received_at - self.baseline_candidate_since).to_sec())
            rospy.logwarn_throttle(
                2.0,
                "Settling before baseline capture (%.2fs left)",
                time_left
            )
            return

        self.try_initialize_slots(stamp)

    def try_initialize_slots(self, stamp: rospy.Time) -> None:
        self.slots.clear()
        for block_id, block in self.latest_observed.items():
            self.slots[block_id] = SlotState(
                block_id=block_id,
                color=block.color,
                x=block.position.x,
                y=block.position.y,
                z=block.position.z,
            )

        self.slots_initialized = True
        self.baseline_candidate_since = rospy.Time(0)
        rospy.loginfo("Initialized %d slots.", len(self.slots))

    def evaluate_slots(self, observed: Dict[int, object], stamp: rospy.Time) -> None:
        if len(observed) == 0:
            if self.all_blocks_missing_since.is_zero():
                self.all_blocks_missing_since = stamp
            elif (stamp - self.all_blocks_missing_since).to_sec() >= self.reset_delay_sec:
                rospy.logwarn("All blocks missing for %.1fs. Resetting slots for a new game.", self.reset_delay_sec)
                self.slots_initialized = False
                self.slots.clear()
                self.all_blocks_missing_since = rospy.Time(0)
                return
        else:
            self.all_blocks_missing_since = rospy.Time(0)

        for slot in self.slots.values():
            current_block = observed.get(slot.block_id)
            slot_occupied = current_block is not None and self.matches_slot(slot, current_block)

            if slot_occupied:
                slot.missing_since = None
                slot.selected_latched = False
                continue

            if slot.selected_latched:
                continue

            if slot.missing_since is None:
                slot.missing_since = stamp
                continue

            held_missing = (stamp - slot.missing_since).to_sec()
            cooled_down = (stamp - self.last_selection_time).to_sec() >= self.selection_cooldown_sec
            
            if held_missing >= self.missing_hold_sec and cooled_down:
                self.publish_selection(slot, stamp)
                slot.selected_latched = True

    def matches_slot(self, slot: SlotState, block: object) -> bool:
        dx = slot.x - block.position.x
        dy = slot.y - block.position.y
        dz = slot.z - block.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz) <= self.max_slot_distance_m

    def publish_selection(self, slot: SlotState, stamp: rospy.Time) -> None:
        msg = PlayerSelection()
        msg.header.stamp = stamp
        msg.header.frame_id = self.target_frame
        
        msg.block_id = slot.block_id
        msg.color = slot.color
        msg.selection_type = self.selection_type
        msg.selection_time = stamp
        msg.confidence = 1.0
        
        self.selection_pub.publish(msg)
        self.last_selection_time = stamp
        
        rospy.loginfo(
            "Selected block %d (%s)",
            slot.block_id,
            slot.color
        )


def main() -> None:
    rospy.init_node("player_selection")
    PlayerSelectionNode()
    rospy.spin()

if __name__ == "__main__":
    main()
