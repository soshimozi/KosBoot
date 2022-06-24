//@LAZYGLOBAL OFF.

IF NOT EXISTS("1:/init.ks") { RUNPATH("0:/init_select.ks"). }
RUNONCEPATH("1:/init.ks").

FOR f IN LIST(
  "lib_runmode.ks",
  "lib_steer.ks"
) { RUNONCEPATH(loadScript(f)). }

pOut("KLaunch.ks v1.0.0 20210716").

IF cOk() { 
	runpath("0:/update.ks"). 
}

// desired inclination for final orbit
LOCAL desiredInclination IS 51.46.
// desired altitude of final orbit
LOCAL desiredApoapsis IS 500.	
// inverted ascent? ('i' for inverted)
LOCAL flightAt IS 0.
// longitude of assending node
LOCAL desiredLAN IS -1.
// desired vertical speed for pitch maneuver
LOCAL desiredVerticalSpeedForPitch IS 50.

	
//*************************
//*** ASCENT PARAMETERS ***
//*************************
   // Desired altitude to begin pitching maneuver.
LOCAL pitchStartingAlt to 250. 
   // Altitude at which pitch will be 45 degrees.
LOCAL halfPitchedAlt to 12000.
   // Desired altitude for thrust to limit.
LOCAL tLimitAlt to 20000.
   // TWR at above altitude
LOCAL thrustAdj to 1.1.
   // TWR for final stage of ascent
LOCAL finalThrustAdj to 0.9.
   // Desired altitude to lock to orbital prograde.
LOCAL lockAlt to 40000.
   // Desired altitude to stage fairings.
LOCAL fairingAlt to 70000.
   // Desired altitude to extend deployables.
LOCAL deployAlt to 80000.
//*************************

   // Current thrust setting.
LOCAL thrustSetting to 1.
   // Thrust limiter.
LOCAL thrustLimiter to 1.
   // Are deployables deployed?
LOCAL deployed to FALSE.
   // Is vessel locked to orbital prograde?
LOCAL proLocked to FALSE.
   // Is throttle limitted?
LOCAL thrustLimited to FALSE.
   // Is upper stage thrust limited?
LOCAL upperLimited TO FALSE.
   // Is abort called?
LOCAL aborted to FALSE.
   // Are fairings staged?
LOCAL fairingStaged to FALSE.
   // Holds vessel's current pitch
LOCAL vPitch to 90.
   // Holds vessel's current heading
LOCAL vHeading to 0.
   // Countdown beeps provided by Hildimar
LOCAL voice to getVoice(0).
LOCAL voiceTickNote to NOTE(480, 0.1).
LOCAL voiceTakeOffNote to NOTE(720, 0.5).

	// set if boosters have been jettisoned - only set IF there are boosters
LOCAL boostersJettisoned TO FALSE.
	// set if there are boosters
LOCAL hasBoosters TO FALSE.
	// tag to look for boosters in the parts list
LOCAL boosterTag TO "Booster".
	// list of boosters (parts with tag boosterTag)
LOCAL boosterPartList TO SHIP:PARTSTAGGED( boosterTag ).

//**********************************

WAIT UNTIL cOk().
RUNPATH("0:/lib_launch_geo.ks").

FUNCTION validTarget {
  RETURN HASTARGET AND TARGET:OBT:BODY = BODY.
}

main().

   // Main program
FUNCTION main {

	UNTIL runMode() = 99 {

		LOCAL rm IS runMode().
		IF rm < 0 {

			SET hasBoosters TO NOT boosterPartList:empty.

			setAbortTrigger().

			calculateLaunchDetails().

			igniteMainEngines().
			
			checkNominalThrust(1, 99).
		}
		ELSE IF rm = 1 {
		
			pitchManuever(2, 99).
		}
		ELSE IF rm = 2 {
			gravityTurn(3, 99).
		} ELSE if rm = 3 {
			meco(4, 99).
		} ELSE IF rm = 4 {
			circNode(5, 99).
		} ELSE IF rm = 5 {
			xMan(400, 99).
		} ELSE IF rm = 400 {

			hudMsg("Mission complete. Hit abort to retry (mode " + abortMode() + ").").

			pOut("Setting SAS to Normal for better sun tracking.").
			SAS ON.
			SET SASMODE TO "NORMAL".

			WAIT UNTIL runMode() <> 400.
		}
	}
}

FUNCTION checkNominalThrust {
	PARAMETER nextMode, abortMode.
	
	// checking for subnominal thrust
	IF (SHIP:AVAILABLETHRUSTAT(1.0) < 1.15*(MASS-clampMass())*CONSTANT:g0) {
		pOut("Subnominal Thrust Detected.").
		pOut("Attempting Shutdown.").
		LOCK THROTTLE to 0.

		SET aborted to TRUE.
		runMode(abortMode).

	} ELSE {
		// proceed with launch
		
		WAIT 0.5. 
		STAGE.
		pOut("Launching now").
		WAIT 0.1.

		runMode(nextMode).
	}  
	
	WAIT 0.5.
}

FUNCTION calculateLaunchDetails {
	hudMsg("Please select a target").
	pOut("Waiting.").
	WAIT UNTIL validTarget().

	pOut("Valid target. Calculating launch details.").
	LOCAL ap IS MIN(85000, MAX(ROUND(TARGET:PERIAPSIS)+250,75000)).
	LOCAL t_I IS TARGET:OBT:INCLINATION.
	LOCAL t_LAN IS TARGET:OBT:LAN.

	LOCAL launch_details IS calcLaunchDetails(ap,t_I,t_LAN).
	LOCAL az IS launch_details[0].
	LOCAL launch_time IS launch_details[1].

	SET desiredInclination TO t_I.
	SET desiredApoapsis TO ap.
	
	pOut(launch_details).
	pOut("Launching in " + launch_time + " seconds.").
	warpToLaunch(launch_time).
}
FUNCTION igniteMainEngines {
	pOut("Initiating launch sequence.  Standby.").

	LOCK STEERING to UP + R(0, 0, 180).
	pOut("Locking attitude control.").
	WAIT 0.5. 
	LOCK THROTTLE to 1.
	pOut("Throttle to full.").
	pOut("Ignition").
	STAGE.
	WAIT 0.5.
}

   // Pitch setting
FUNCTION myPitch {
   RETURN 90*halfPitchedAlt / (ALTITUDE + halfPitchedAlt).
}

   // Roll setting
FUNCTION myRoll {
   IF (flightAt = "i") {
      SET tempRoll to 270 - myHeading.
   } ELSE {
      SET tempRoll to 360 - myHeading.
   }
   RETURN tempRoll.
}

   //Heading setting
FUNCTION myHeading {
   SET roughHeading to 90 - desiredInclination.
   IF (roughHeading < 0) {
      SET roughHeading to 360 + roughHeading.
   }
      //taking into account Kerbin's rotation
   SET triAng to abs(90 - roughHeading).
      //vH calculation assumes orbital speed of 1320 m/s when vessel locks to prograde
   SET vH to sqrt(1774800 - 475200*cos(triAng)).
   SET correction to arcsin(180*sin(triAng) / vH).
   IF (desiredInclination > 0) {
      SET correction to -1*correction.
   } 
   IF ((roughHeading + correction) < 0) { 
      RETURN roughHeading + correction + 360.
   } ELSE {
      RETURN roughHeading + correction.
   }   
}



   //PITCHING MANEUVER
FUNCTION pitchManuever {
	PARAMETER nextMode, abortMode.
	
	LOCK vPitch to 90 - VANG(UP:FOREVECTOR, FACING:FOREVECTOR).
	LOCK vHeading to MOD(360 - LATLNG(90, 0):BEARING,360).
   
	UNTIL(SHIP:VERTICALSPEED > desiredVerticalSpeedForPitch) {
		checkBoosters().
	  
		IF ABS(vPitch-myPitch())>10 AND NOT aborted {
			autoAbort().
			runMode(abortMode).
			BREAK.
		}
		
		WAIT 0.1.  
	}
	
	if NOT aborted {
		pOut("Starting pitching maneuver.").
		SET initialHeading to myHeading().
		SET initialRoll to myRoll().
		LOCK STEERING to HEADING(initialHeading, myPitch())+ R(0, 0, initialRoll).
		WAIT 2.
   
		runMode(nextMode).
	}
}

   //LOCKING TO PROGRADE
FUNCTION lockToPrograde {
   pOut("Locking to prograde.").
   LOCK STEERING to PROGRADE + R(0, 0, myRoll()).
   SET proLocked to TRUE.
}

   //Returns ship TWR
FUNCTION shipTWR {
   RETURN AVAILABLETHRUST*thrustSetting / (MASS*CONSTANT:g0).
}

   //LIMITS TWR to thrustAdj Setting
FUNCTION limitThrust {
      //force of gravity on vessel
   LOCK Fg to (BODY:MU/(BODY:RADIUS+ALTITUDE)^2)*MASS.
      //locks thrust
   IF (AVAILABLETHRUST > 0) {
      IF NOT thrustLimited {
         SET thrustSetting to thrustAdj*Fg / (AVAILABLETHRUST+0.001).
         pOut("Adjusting TWR to " + thrustAdj).
      } ELSE {
         SET thrustSetting to finalThrustAdj*Fg / (AVAILABLETHRUST+0.001).
         pOut("Adjusting TWR to " + finalThrustAdj).
      }
      LOCK THROTTLE to thrustSetting.
      IF thrustLimited {
         SET upperLimited to TRUE.
      }
      SET thrustLimited to TRUE.
   } ELSE {
	  pOut("No thrust detected.  Atempting staging.").
      STAGE.
      WAIT 0.1.
   }
}

   //Main Engine Cut-off
FUNCTION meco {
	PARAMETER nextMode, abortMode.
	
	LOCK THROTTLE to 0.
	pOut("Engine Cut-off.").
	WAIT UNTIL ALTITUDE > lockAlt.
	IF NOT proLocked {
		lockToPrograde().
	}
	WAIT UNTIL ALTITUDE > fairingAlt.
	IF (ALTITUDE > fairingAlt) AND NOT fairingStaged {
		autoFairing().
	}	
	  
	WAIT UNTIL ALTITUDE > deployAlt.
	IF NOT deployed {
		autoDeploy().
	}
	
	IF NOT aborted {
		runMode(nextMode).
	}
}

   //Looks for events until MECO.
FUNCTION gravityTurn {
	
	PARAMETER nextMode, abortMode.
	UNTIL (APOAPSIS > desiredApoapsis*1000) {
		IF (ALTITUDE > lockAlt) AND NOT proLocked {
			lockToPrograde().
		}
		IF (ALTITUDE > tLimitAlt) AND NOT thrustLimited {
			limitThrust().
		}
		IF (shipTWR() < thrustAdj - 0.1) AND thrustLimited AND NOT upperLimited {
			limitThrust().
		}
		IF (ALTITUDE > deployAlt) AND NOT deployed {
			autoDeploy().
		}
		IF (ALTITUDE > fairingAlt) AND NOT fairingStaged {
			autoFairing().
		}
		IF (desiredInclination<80 OR desiredInclination>100) {
			IF (ALTITUDE < lockAlt ) AND  NOT aborted {
				IF (ABS(vPitch-myPitch())>60) OR (ABS(vHeading-myHeading()) > 60) {
					autoAbort().
					runMode(abortMode).
					BREAK.
				}
			} ELSE {
				IF (VANG(FACING:FOREVECTOR, PROGRADE:FOREVECTOR)>60  AND  NOT aborted) {
					autoAbort().
					runMode(abortMode).
					BREAK.
				}      
			}
		}
	  
		checkBoosters().
	  
		WAIT 0.1.
	}
	
	IF NOT aborted {
		runMode(nextMode).
	}
}   

FUNCTION checkBoosters {
	IF NOT boostersJettisoned AND hasBoosters {
	
		//LOCAL partList TO SHIP:PARTSNAMED( "liquidEngine" ).
		LOCAL totalThrust TO 0.0.
		FOR P IN boosterPartList {
			LOCAL MOD TO P:GETMODULE("ModuleEnginesFX").
			SET totalThrust TO totalThrust + MOD:GETFIELD("Thrust").
		}     

		IF (totalThrust <= 0) {
			pOut("Booster thrust loss detected").
			pOut("Intiating booster jettison").
			
			AG2 ON.	 // jettison
			SET boostersJettisoned TO TRUE.
			
			RETURN TRUE.
		}
		
		RETURN FALSE.
	}
	
	RETURN TRUE.
}
   //Aborts launch
FUNCTION autoAbort {
   LOCK THROTTLE to 0.
   IF (ALTITUDE < tLimitAlt) {
      ABORT ON.
   }
   pOut("Attitude control loss detected.").
   pOut("ABORTING!").  

   runMode(99).   
   SET aborted to TRUE.
}
   //Stages fairings
FUNCTION autoFairing {
   SET fairing to false.      //holds if vessel has a fairing
   LIST PARTS IN partList.    //LIST of parts on vessel
   FOR part IN partList {     //checks if fairing in partList
      IF (part:NAME = "fairingSize1" OR
	      part:NAME = "fairingSize2" OR
		  part:NAME = "fairingSize3" OR
		  part:NAME = "restock-fairing-base-0625-1" OR
		  part:NAME = "restock-fairing-base-1875-1" OR
		  part:NAME = "fairingSize1p5" OR
		  part:NAME = "fairingSize4") {
		     SET fairing to true.
           BREAK.
		  }
   }              //stages if vessel has a fairing
   IF fairing {
      AG5 ON.
      pOut("Staging Fairing.").
   }
   SET fairingStaged to TRUE.
}

   //Deploys equipment
FUNCTION autoDeploy {
        //all deployable equipment must be on action group 10
   AG10 ON.
   LIGHTS ON.
   pOut("Extending deployable equipment.").
   SET deployed to TRUE.
}

   //Watches for abort trigger
FUNCTION setAbortTrigger {
   ON ABORT {
         IF NOT aborted {
            LOCK THROTTLE to 0.
            ABORT ON.
            pOut( "Abort has been triggered manually.").
            pOut( "ABORTING!").  
			
			runMode(99).
            SET aborted to TRUE.
         }
   }
}

   //Returns mass of launch clamps
   //***THIS METHOD ALWAYS RETURNS ZERO***
FUNCTION clampMass {
   LIST PARTS IN partList.    //LIST of parts on vessel
   SET cMass to 0.
   FOR part IN partList {     //checks if part is a launch clamp
      IF (part:NAME = "launchClamp1") {
		     SET cMass to cMass + part:MASS.
		}
      RETURN cMass.
   }   
}
   // creates node at apoapsis to circularize
FUNCTION circNode {
	PARAMETER nextMode, abortMode.
	
	WAIT UNTIL (ALTITUDE > 70000).
	SET futureVelocity to SQRT(VELOCITY:ORBIT:MAG^2-2*BODY:MU*(1/(BODY:RADIUS+ALTITUDE) - 1/(BODY:RADIUS+ORBIT:APOAPSIS))).
	SET circVelocity to SQRT(BODY:MU/(ORBIT:APOAPSIS+BODY:RADIUS)).
	SET newNode to NODE(TIME:SECONDS+ETA:APOAPSIS, 0, 0, circVelocity-futureVelocity).
	ADD newNode.
	pOut("Circularization burn plotted.").
	
	IF NOT aborted {
		runMode(nextMode).
	}
}

   // Executes next maneuver
FUNCTION xMan {
	PARAMETER nextMode, abortMode.
	
	SAS OFF.
      // amount of time before end to reduce burn
	SET startReduceTime to 2.
      // desired TWR
	SET desiredTWR to 0.9.
      // holds data regarding maneuver node
	SET mNode to NEXTNODE.
      // start time of burn
	SET TWR to AVAILABLETHRUST / MASS*CONSTANT:g0.
      // limiting thrust if TWR is more than desired TWR
	IF (TWR > desiredTWR) {
      SET thrustLimiter to desiredTWR*MASS*CONSTANT:g0 / AVAILABLETHRUST.
	}
	SET startTime to calculateStartTime(mNode, startReduceTime).
      // start direction of burn
	SET startVector to mNode:BURNVECTOR.
	lockSteering(mNode).
	startBurn(startTime).
	WAIT UNTIL burnTime(mNode) < startReduceTime.
	reduceThrottle().
	endBurn(mNode, startVector).
	
	runMode(nextMode).
}

   // calculates the start time of the burn
FUNCTION calculateStartTime {
   PARAMETER mNode.
   PARAMETER startReduceTime.
   RETURN TIME:SECONDS + mNode:ETA - halfBurnTime(mNode) - startReduceTime/2.
}

   // locks attitude to the burn vector
FUNCTION lockSteering {
   PARAMETER mNode.
   LOCK STEERING to mNode:BURNVECTOR.
   pOut("Locking attitude to burn vector.").
}

   // maneuver ends when burn vector deviates by more than 3.5 degrees
FUNCTION maneuverComplete {
   PARAMETER mNode.
   PARAMETER startVector.
   RETURN VANG(startVector, mNode:BURNVECTOR) > 3.5.
}

   // calculates how long the burn will take
FUNCTION burnTime {
   PARAMETER mNode.
   SET bTime to -1.
   SET delV to mNode:BURNVECTOR:MAG. 
   SET finalMass to MASS / (CONSTANT:E^(delV/(currentISP()*CONSTANT:g0))).
   //SET startAcc to AVAILABLETHRUST / MASS.
   //SET finalAcc to AVAILABLETHRUST / finalMass.
      // checking to make sure engines haven't flamed out
   IF (AVAILABLETHRUST > 0) {
      //SET bTime to 2*delV / (startAcc+finalAcc).
      SET bTime to delV*(MASS - finalMass) / thrustLimiter / AVAILABLETHRUST / LN(MASS/finalMass).
   } 
   RETURN bTime.        
}

   // calculates how long to do half the burn
FUNCTION halfBurnTime {
   PARAMETER mNode.
   SET bTime to -1.
   SET delV to mNode:BURNVECTOR:MAG/2. 
   SET finalMass to MASS / (CONSTANT:E^(delV/(currentISP()*CONSTANT:g0))).
   //SET startAcc to AVAILABLETHRUST / MASS.
   //SET finalAcc to AVAILABLETHRUST / finalMass.
      // checking to make sure engines haven't flamed out
   IF (AVAILABLETHRUST > 0) {
      //SET bTime to 2*delV / (startAcc+finalAcc).
      SET bTime to delV*(MASS - finalMass) / thrustLimiter / AVAILABLETHRUST / LN(MASS/finalMass).
   } 
   RETURN bTime.        
}

   // calculates the combined ISP of active engines
FUNCTION currentISP {
   LIST ENGINES in engineList.
   SET sumOne to 0.
   SET sumTwo to 0.
   FOR eng in engineList {
      IF eng:IGNITION {
         SET sumOne to sumOne + eng:AVAILABLETHRUST.
         SET sumTwo to sumTwo + eng:AVAILABLETHRUST/eng:ISP.
      }
   }
   IF (sumTwo > 0) {
      RETURN sumOne / sumTwo.
      // returns -1 if no active engines
   } ELSE {
      RETURN -1.
   }
}

   // reduce throttle function
FUNCTION reduceThrottle {
   pOut("Reducing throttle.").
   SET reduceTime to startReduceTime*(-1)*LN(0.1)/0.9.
   SET startTime to TIME:SECONDS - 0.5.
   SET stopTime to TIME:SECONDS + reduceTime - 0.5.
   SET scale to CONSTANT:E^(-0.9/startReduceTime).
   LOCK THROTTLE to thrustLimiter*scale^(TIME:SECONDS - startTime).
   WAIT UNTIL TIME:SECONDS > stopTime.
   LOCK THROTTLE to 0.1.
}

   // starts the burn
FUNCTION startBurn {
   PARAMETER startTime.
   pOut("Circularization burn to start in " + ROUND(startTime - TIME:SECONDS) + " seconds.").
   warpToManeuverNode(startTime-10).
   
   //WAIT UNTIL TIME:SECONDS > (startTime-10).
      // 30 second countdown
   pOut("Starting burn in 10 seconds").
   WAIT 5.
   pOut("Starting burn in 5 seconds").
   WAIT 1.
   pOut("Starting burn in 4 seconds").
   WAIT 1.
   pOut("Starting burn in 3 seconds").
   WAIT 1.
   pOut("Starting burn in 2 seconds").
   WAIT 1.
   pOut("Starting burn in 1 seconds").
   WAIT 1.
   pOut("Engaging engines.").
   LOCK THROTTLE to thrustLimiter.
}

   // kills throttle when burn is complete
FUNCTION endBurn {
   PARAMETER mNode.
   PARAMETER startVector.
   WAIT UNTIL maneuverComplete(mNode, startVector).
   pOut("Burn Complete.").

   LOCK THROTTLE to 0.
   UNLOCK STEERING.
   SAS ON.
   REMOVE mNode.
   WAIT 1.
   SWITCH TO 1.  
   pOut("Switching volume to vessel.").
   WAIT 2.
}	

FUNCTION warpToManeuverNode
{
  PARAMETER startTime.
  IF startTime - TIME:SECONDS > 30 {
    pOut("Waiting for maneuver node.").
    UNTIL WARPMODE = "RAILS" { SET WARPMODE TO "RAILS". SET WARP TO 1. WAIT 0.2. }
    doWarp(startTime).
  }
}
