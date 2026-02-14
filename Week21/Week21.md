## Week 21 Driving the Robot (Failed Again)

https://github.com/user-attachments/assets/460ef269-b79e-4592-b1dc-e5c65513d691

This week was mostly about debugging physics, joints, and articulation issues in Isaac Sim. Below is the **cleaned and corrected log**, keeping the **same logic and steps**, but with clearer structure and wording.

---

## 1. Fixing Last Week’s Errors

### Action 1 – Enable Rigid Body on the Chassis

* Select `/Root/nova_carter/chassis_link` in the **Stage**.
* Open the **Properties** tab.
* Scroll down.
* If there is no **Rigid Body** section:

  * Click `+ Add > Physics > Rigid Body`.
* **Crucial:** Make sure **Rigid Body Enabled** is checked.

---

### Action 2 – Action Graph Path Consistency

* The **Action Graph Controller** must point to the **exact same folder** that contains the **Articulation Root** (`nova_carter`).

After this fix, a new error appeared:

```
2026-01-04 14:40:34 [149,588ms] [Error] [omni.physx.plugin] PhysicsUSD: CreateJoint - no bodies defined at body0 and body1, joint prim: /Root/rs007n_onrobot_rg2/FixedJoint
```

---

## 2. Fixing the Broken Fixed Joint

### Action 3 – Rebinding Joint Bodies

* Locate the joint mentioned in the error:
  `/Root/rs007n_onrobot_rg2/FixedJoint`
* Select it in the **Stage Tree**.

#### Body 0 (Chassis)

* Click **Add Target** (or clear the existing target first).
* Select:
  `/Root/nova_carter/chassis_link`

#### Body 1 (Arm Base)

* Click **Add Target**.
* Select:
  `/Root/rs007n_onrobot_rg2/base_link`
  *(or `onrobot_rg2_base_link`, depending on the model)*

---

## 3. Action Graph Mismatch

### Action 4 – Observations and Physics Fix

* The Omnigraph being used is **not the intended one**.
* Plan: switch to the other OG that we actually want to follow.
* This removes errors but introduces **warnings**.

#### Physics Fix – Add Mass to the Robot

* Select `/Root/nova_carter/chassis_link`.
* Go to **Properties > Physics**.
* If there is no **Mass** section:

  * Click `+ Add > Physics > Mass`.
* Set **Mass = 60 kg**.

---

## 4. Root Cause Found: Wrong Nova Carter Version

I realized that I was using the **wrong Nova Carter asset**.

* I should have been using the **source version**, which already includes:

  * Proper joints
  * Embedded action graphs

Because of this:

* I switched to the correct **`nova_carter_ros`** robot.
* The arm needs to be linked **again** to this new robot.

---

## 5. Performance Constraints and GPU Fixes

Due to limited GPU VRAM, the following optimizations were applied.

### Rendering Optimization

* Go to **Render Settings > Ray Tracing**.
* Disable:

  * **Reflections**

Reason: physics is required, not visual quality.

---

## 6. Verifying the Fixed Joint Again

* Locate the **Fixed Joint** connecting the Nova Carter and the arm.

Verify targets:

* **Body 0:** `/Root/nova_carter/chassis_link`
* **Body 1:** `/Root/rs007n_onrobot_rg2/base_link`

### Conflict Check

* Ensure the arm base (`base_link`):

  * Does **not** have a Rigid Body set to **Static / Kinematic**.
  * It must be **Dynamic**, otherwise it cannot be attached properly.

After this step, **Isaac Sim crashed**.

---

## 7. Final Fix – Correct Assembly Method

### Action 5 – Using Robot Assembler (Correct Way)

#### Enable the Tool

* Go to:
  `Tools > Robotics > Asset Editors > Robot Assembler`

#### Configure the Assembler

* **Base Robot:** `/Root/nova_carter`
* **Attach Robot:** `/Root/rs007n_onrobot_rg2`
* **Base Robot Attach Point:** `chassis_link`
* **Attach Robot Attach Point:** `base_link`

#### Critical Checkbox

* Enable:
  **Single Articulation** (or **Treat as single robot**)

* Click **Assemble**.

#### Result

* Extra Rigid Body physics on the arm base is removed.
* Extra Articulation Root is deleted.
* Hierarchy and parenting are corrected automatically.

✅ Expected result achieved.

---

## 8. Switching to Script Base Mode

Due to GPU memory limitations:

* Switched to **script-based execution**.
* This significantly reduces GPU load.

### Notes

* The script (`running_the_sim`) works correctly.

* Startup time is long (~5 minutes) due to:

  * Physics initialization
  * Logging

* Logs can be removed if not needed.

The script still **opens Isaac Sim**, because:

* Visual feedback is necessary to understand robot behavior.
* RViz-only operation is possible but would require script changes.

---

## 9. Current Status

* Robot movement is possible but requires more tuning.
* Progress is slow due to hardware limits.

That’s it for this week.

See you next week 🤖
