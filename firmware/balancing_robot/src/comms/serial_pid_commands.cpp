#include "../config/config.h"  // FIRST: ENABLE_CASCADE_PID / CASCADE_MODE_ANGLE_CONTROL
#include "serial_commands.h"  // KP.., cascade vars, savePIDToPreferences(), lastPIDUpdateTime
#include "serial_pid_commands.h"
#include "../control/cascade_pid_controller.h"  // pitch/roll_rate_target (rate mode)

// ===== PID-tuning commands (moved from handleSerialCommand() verbatim) =====
// Returns true when the command was handled.
bool handlePIDCommand(const String& command) {
    if (command.startsWith("set p ")) {
      float value = command.substring(6).toFloat();
      KP = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[PID] KP updated to: "); Serial.println(KP);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("set i ")) {
      float value = command.substring(6).toFloat();
      KI = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[PID] KI updated to: "); Serial.println(KI);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("set d ")) {
      float value = command.substring(6).toFloat();
      KD = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[PID] KD updated to: "); Serial.println(KD);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    // PITCH AXIS TUNING (separate)
    else if (command.startsWith("pitch_p ") || command.startsWith("pp ")) {
      float value = (command.indexOf("pitch_p") >= 0) ? command.substring(8).toFloat() : command.substring(3).toFloat();
      KP_Pitch = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[PITCH PID] KP_Pitch updated to: "); Serial.println(KP_Pitch);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("pitch_i ") || command.startsWith("pi ")) {
      float value = (command.indexOf("pitch_i") >= 0) ? command.substring(8).toFloat() : command.substring(3).toFloat();
      KI_Pitch = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[PITCH PID] KI_Pitch updated to: "); Serial.println(KI_Pitch);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("pitch_d ") || command.startsWith("pd ")) {
      float value = (command.indexOf("pitch_d") >= 0) ? command.substring(8).toFloat() : command.substring(3).toFloat();
      KD_Pitch = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[PITCH PID] KD_Pitch updated to: "); Serial.println(KD_Pitch);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    // ROLL AXIS TUNING (separate)
    else if (command.startsWith("roll_p ") || command.startsWith("rp ")) {
      float value = (command.indexOf("roll_p") >= 0) ? command.substring(7).toFloat() : command.substring(3).toFloat();
      KP_Roll = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[ROLL PID] KP_Roll updated to: "); Serial.println(KP_Roll);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("roll_i ") || command.startsWith("ri ")) {
      float value = (command.indexOf("roll_i") >= 0) ? command.substring(7).toFloat() : command.substring(3).toFloat();
      KI_Roll = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[ROLL PID] KI_Roll updated to: "); Serial.println(KI_Roll);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("roll_d ") || command.startsWith("rd ")) {
      float value = (command.indexOf("roll_d") >= 0) ? command.substring(7).toFloat() : command.substring(3).toFloat();
      KD_Roll = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[ROLL PID] KD_Roll updated to: "); Serial.println(KD_Roll);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command == "reset_pid") {
      resetPIDToDefaults();
    }
    // YAW AXIS Tuning (Short form: yp, yi, yd)
    else if (command.startsWith("yaw_p ") || command.startsWith("yp ")) {
      float value = (command.indexOf("yaw_p") >= 0) ? command.substring(6).toFloat() : command.substring(3).toFloat();
      KP_Yaw = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[YAW PID] KP_Yaw updated to: "); Serial.println(KP_Yaw);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("yaw_i ") || command.startsWith("yi ")) {
      float value = (command.indexOf("yaw_i") >= 0) ? command.substring(6).toFloat() : command.substring(3).toFloat();
      KI_Yaw = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[YAW PID] KI_Yaw updated to: "); Serial.println(KI_Yaw);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("yaw_d ") || command.startsWith("yd ")) {
      float value = (command.indexOf("yaw_d") >= 0) ? command.substring(6).toFloat() : command.substring(3).toFloat();
      KD_Yaw = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[YAW PID] KD_Yaw updated to: "); Serial.println(KD_Yaw);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("yaw_rate ") || command.startsWith("yr ")) {
      float value = (command.indexOf("yaw_rate") >= 0) ? command.substring(9).toFloat() : command.substring(3).toFloat();
      yaw_rate_target = value;
      Serial.print("\n[YAW RATE] Target Rate set to: "); Serial.print(yaw_rate_target);
      Serial.println(" deg/s (Rate-Only Control)");
    }
    #if ENABLE_CASCADE_PID && !CASCADE_MODE_ANGLE_CONTROL
    // Rate mode commands (only available in cascade rate mode)
    else if (command.startsWith("pitch_rate ") || command.startsWith("pr ")) {
      float value = (command.indexOf("pitch_rate") >= 0) ? command.substring(11).toFloat() : command.substring(3).toFloat();
      pitch_rate_target = value;
      Serial.print("\n[CASCADE PITCH RATE] Target Rate set to: "); Serial.print(pitch_rate_target);
      Serial.println(" deg/s");
    }
    else if (command.startsWith("roll_rate ") || command.startsWith("rr ")) {
      float value = (command.indexOf("roll_rate") >= 0) ? command.substring(10).toFloat() : command.substring(3).toFloat();
      roll_rate_target = value;
      Serial.print("\n[CASCADE ROLL RATE] Target Rate set to: "); Serial.print(roll_rate_target);
      Serial.println(" deg/s");
    }
    #endif
    else if (command == "load") {
      lastPIDUpdateTime = millis();
      Serial.println("\n===== Current Dual-Axis PID Values =====");
      Serial.println("PITCH AXIS:");
      Serial.print("  KP_Pitch: "); Serial.println(KP_Pitch);
      Serial.print("  KI_Pitch: "); Serial.println(KI_Pitch);
      Serial.print("  KD_Pitch: "); Serial.println(KD_Pitch);
      Serial.println("ROLL AXIS:");
      Serial.print("  KP_Roll: "); Serial.println(KP_Roll);
      Serial.print("  KI_Roll: "); Serial.println(KI_Roll);
      Serial.print("  KD_Roll: "); Serial.println(KD_Roll);
      Serial.println("YAW AXIS (Rate-Only Control):");
      Serial.print("  KP_Yaw: "); Serial.println(KP_Yaw);
      Serial.print("  KI_Yaw: "); Serial.println(KI_Yaw);
      Serial.print("  KD_Yaw: "); Serial.println(KD_Yaw);
      #if ENABLE_CASCADE_PID
      Serial.println("\n*** CASCADE PID MODE  ***");
      #if CASCADE_MODE_ANGLE_CONTROL
      Serial.println("Current Mode: ANGLE CONTROL");
      #else
      Serial.println("Current Mode: RATE CONTROL");
      Serial.println("RATE TARGETS:");
      Serial.print("  pitch_rate_target: "); Serial.println(pitch_rate_target);
      Serial.print("  roll_rate_target: "); Serial.println(roll_rate_target);
      #endif
      Serial.println("PITCH AXIS - ANGLE PID (Outer Loop):");
      Serial.print("  cascade_pitch_angle_kp: "); Serial.println(cascade_pitch_angle_kp);
      Serial.print("  cascade_pitch_angle_ki: "); Serial.println(cascade_pitch_angle_ki);
      Serial.print("  cascade_pitch_angle_kd: "); Serial.println(cascade_pitch_angle_kd);
      Serial.println("PITCH AXIS - RATE PID (Inner Loop):");
      Serial.print("  cascade_pitch_rate_kp: "); Serial.println(cascade_pitch_rate_kp);
      Serial.print("  cascade_pitch_rate_ki: "); Serial.println(cascade_pitch_rate_ki);
      Serial.print("  cascade_pitch_rate_kd: "); Serial.println(cascade_pitch_rate_kd);
      Serial.println("ROLL AXIS - ANGLE PID (Outer Loop):");
      Serial.print("  cascade_roll_angle_kp: "); Serial.println(cascade_roll_angle_kp);
      Serial.print("  cascade_roll_angle_ki: "); Serial.println(cascade_roll_angle_ki);
      Serial.print("  cascade_roll_angle_kd: "); Serial.println(cascade_roll_angle_kd);
      Serial.println("ROLL AXIS - RATE PID (Inner Loop):");
      Serial.print("  cascade_roll_rate_kp: "); Serial.println(cascade_roll_rate_kp);
      Serial.print("  cascade_roll_rate_ki: "); Serial.println(cascade_roll_rate_ki);
      Serial.print("  cascade_roll_rate_kd: "); Serial.println(cascade_roll_rate_kd);
      Serial.println("(Values persisted in NVS)");
      #endif
      Serial.println("=========================================\n");
      Serial.println("Telemetry paused for 5 seconds...");
    }

#if ENABLE_CASCADE_PID
    // ===== CASCADE PID TUNING COMMANDS - PITCH AXIS =====
    // Cascade Angle PID (Outer Loop) - PITCH
    else if (command.startsWith("app ")) {
      String valueStr = command.substring(4);
      valueStr.trim();  // Remove any leading/trailing whitespace
      float value = valueStr.toFloat();
      if (value == 0.0f && valueStr != "0" && valueStr != "0.0") {
        Serial.println("[ERROR] Failed to parse value: " + valueStr);
        Serial.println("[HELP] Correct format: app <float_value>  (e.g., app 60.0)");
      } else {
        cascade_pitch_angle_kp = value;
        lastPIDUpdateTime = millis();
        savePIDToPreferences();
        Serial.print("[CASCADE-PITCH] Angle KP updated to: "); Serial.println(cascade_pitch_angle_kp);
      }
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("api ")) {
      float value = command.substring(4).toFloat();
      cascade_pitch_angle_ki = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-PITCH] Angle KI updated to: "); Serial.println(cascade_pitch_angle_ki);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("apd ")) {
      float value = command.substring(4).toFloat();
      cascade_pitch_angle_kd = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-PITCH] Angle KD updated to: "); Serial.println(cascade_pitch_angle_kd);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    // Cascade Rate PID (Inner Loop) - PITCH
    else if (command.startsWith("rpp ")) {
      float value = command.substring(4).toFloat();
      cascade_pitch_rate_kp = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-PITCH] Rate KP updated to: "); Serial.println(cascade_pitch_rate_kp);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("rpi ")) {
      float value = command.substring(4).toFloat();
      cascade_pitch_rate_ki = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-PITCH] Rate KI updated to: "); Serial.println(cascade_pitch_rate_ki);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("rpd ")) {
      float value = command.substring(4).toFloat();
      cascade_pitch_rate_kd = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-PITCH] Rate KD updated to: "); Serial.println(cascade_pitch_rate_kd);
      Serial.println("Telemetry paused for 5 seconds...");
    }

    // ===== CASCADE PID TUNING COMMANDS - ROLL AXIS =====
    // Cascade Angle PID (Outer Loop) - ROLL
    else if (command.startsWith("arp ")) {
      float value = command.substring(4).toFloat();
      cascade_roll_angle_kp = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-ROLL] Angle KP updated to: "); Serial.println(cascade_roll_angle_kp);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("ari ")) {
      float value = command.substring(4).toFloat();
      cascade_roll_angle_ki = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-ROLL] Angle KI updated to: "); Serial.println(cascade_roll_angle_ki);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("ard ")) {
      float value = command.substring(4).toFloat();
      cascade_roll_angle_kd = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-ROLL] Angle KD updated to: "); Serial.println(cascade_roll_angle_kd);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    // Cascade Rate PID (Inner Loop) - ROLL
    else if (command.startsWith("rrp ")) {
      float value = command.substring(4).toFloat();
      cascade_roll_rate_kp = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-ROLL] Rate KP updated to: "); Serial.println(cascade_roll_rate_kp);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("rri ")) {
      float value = command.substring(4).toFloat();
      cascade_roll_rate_ki = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-ROLL] Rate KI updated to: "); Serial.println(cascade_roll_rate_ki);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("rrd ")) {
      float value = command.substring(4).toFloat();
      cascade_roll_rate_kd = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-ROLL] Rate KD updated to: "); Serial.println(cascade_roll_rate_kd);
      Serial.println("Telemetry paused for 5 seconds...");
    }

#endif
    else {
      return false;
    }
    return true;
}
