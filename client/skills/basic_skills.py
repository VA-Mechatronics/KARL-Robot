from common.target import Target
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from client.client_api import RobotAPI


class BasicSkills:
    """High-Level Skill Library. Mixin to RobotAPI"""

    def pick(self: "RobotAPI", target: Target, approach: Target = None, open_gripper: float = 100) -> None:
        """Pick sequence with optional approach position.

        w/ approach (suggested)
            1. moveJ to approach and open gripper
            2. moveL to target
            3. close gripper
            4. moveL to approach

        w/o approach
            1. open gripper
            2. moveJ to target
            3. close gripper

        Args:
            target: Target Object to pick.
            approach: Optional Target Object e.g. use target.offset().
            open_gripper: optional width of gripper while open.

        Examples:
            >>> # suggested w/ approach
            >>> robot.pick(target=custom_target, approach=custom_target.offset(z=-50.0), open_gripper=100)
            >>> # minimal
            >>> robot.pick(target=custom_target)
        """
        if approach is None:
            self.gripper(open_gripper)
            self.moveJ(target)
            self.gripper(0)
        else:
            self.moveJ(approach, gripper=open_gripper)
            self.moveL(target)
            self.gripper(0)
            self.moveL(approach)

    def place(self: "RobotAPI", target: Target, approach: Target = None, open_gripper: float = 100) -> None:
        """Place sequence with optional approach position.

        w/ approach (suggested)
            1. moveJ to approach
            2. moveL to target
            3. open gripper
            4. moveL to approach

        w/o approach
            2. moveJ to target
            3. open gripper

        Args:
            target: Target Object to pick
            approach: Optional Target Object e.g. use target.offset()
            open_gripper: optional width of gripper while open

        Examples:
            >>> # suggested w/ approach
            >>> robot.place(target=custom_target, approach=custom_target.offset(z=-50.0), open_gripper=100)
            >>> # minimal
            >>> robot.place(target=custom_target)
        """
        if approach is None:
            self.moveJ(target)
            self.gripper(open_gripper)
        else:
            self.moveJ(approach)
            self.moveL(target)
            self.gripper(open_gripper)
            self.moveL(approach)

    def wrist_flip(self: "RobotAPI", flip: bool = False, target: Target = None) -> None:
        """Do Wrist Flip while optional moving to Target

        Args:
            flip: True = "flip", False = "no_flip". Default False
            target: optinal Target Object. Default act_target

        Examples:
            >>> robot.wrist_flip(flip=True, target=custom_target)
        """
        new_target = target.copy() if target else self.act_target.copy()
        new_target.wrist = "flip" if flip else "no_flip"
        with self.blocking_mode():
            self.moveJ(new_target)
