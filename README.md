# ROXBOTIX Simplified Swerve Drive 2022
This branch of the Swerve2022-Command GitHub Page is dedicated to a **simpler version** of Swerve Drive,
designed for showcasing the robot, going to off-season events, etc. Unlike the other version of this code, it
is lacking many features that are not needed for this specific use case. Some changes are including:

- Nearly every autonomous function has been removed as it was not needed, including CenterShooterToHub, 
  AutoShootCMD, and all Camera references.

  
- The teleop features that were left have been overhauled and significantly simplified where possible with the 
  exception to Swerve Drive. Good examples being the Storage and Shooter mechanisms where they are now more 
  user-controlled, by **holding** a button instead of having it being **fully automatic.** 


- The code has been inspected and mostly documented, some flaws were hopefully fixed and the bugs in this code
  should hopefully be reduced.
