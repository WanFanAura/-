package net.augustus.utils;

import net.minecraft.client.Minecraft;
import net.minecraft.entity.Entity;
import net.minecraft.entity.EntityLivingBase;
import net.minecraft.util.AxisAlignedBB;
import net.minecraft.util.MathHelper;
import net.minecraft.util.Vec3;

import java.security.SecureRandom;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class QuaternionRotationUtils {
    private static final Minecraft mc = Minecraft.getMinecraft();
    private static final SecureRandom random = new SecureRandom();
    private static final Random seedRandom = new Random();
    private static final List<double[]> positionHistory = new ArrayList<>();
    public static long lastRotationTime = 0;
    public static long startTime = System.currentTimeMillis();
    public static long lastTargetSwitchTime = 0;
    private static long lastSeedUpdate = 0;
    public static long lastResetTime = 0;
    private static EntityLivingBase lastTarget = null;
    private static float fixedRandomHeightOffset = 0;
    private static boolean isKillAuraEnabled = false;
    private static int targetCount = 0;
    public static float[] lastTargetRotations = new float[]{0, 0};
    public static long targetSwitchStartTime = 0;

    public static void setKillAuraEnabled(boolean enabled) {
        isKillAuraEnabled = enabled;
        if (enabled) {
            fixedRandomHeightOffset = 0;
            lastSeedUpdate = 0;
            lastTarget = null;
            targetCount = 0;
            startTime = System.currentTimeMillis();
            lastTargetSwitchTime = 0;
            lastResetTime = 0;
            targetSwitchStartTime = 0;
            lastTargetRotations = new float[]{0, 0};
        }
    }

    public static void notifyTargetSwitch() {
        targetCount++;
        lastTargetSwitchTime = System.currentTimeMillis();
        targetSwitchStartTime = lastTargetSwitchTime;
    }

    public static class Octonion {
        public double e0, e1, e2, e3, e4, e5, e6, e7;

        public Octonion(double e0, double e1, double e2, double e3, double e4, double e5, double e6, double e7) {
            this.e0 = e0; this.e1 = e1; this.e2 = e2; this.e3 = e3;
            this.e4 = e4; this.e5 = e5; this.e6 = e6; this.e7 = e7;
        }

        public Octonion normalize() {
            double mag = Math.sqrt(e0 * e0 + e1 * e1 + e2 * e2 + e3 * e3 + e4 * e4 + e5 * e5 + e6 * e6 + e7 * e7);
            return new Octonion(e0 / mag, e1 / mag, e2 / mag, e3 / mag, e4 / mag, e5 / mag, e6 / mag, e7 / mag);
        }

        public Octonion multiply(Octonion other) {
            double a0 = e0 * other.e0 - e1 * other.e1 - e2 * other.e2 - e3 * other.e3 - e4 * other.e4 - e5 * other.e5 - e6 * other.e6 - e7 * other.e7;
            double a1 = e0 * other.e1 + e1 * other.e0 + e2 * other.e3 - e3 * other.e2 + e4 * other.e5 - e5 * other.e4 - e6 * other.e7 + e7 * other.e6;
            double a2 = e0 * other.e2 - e1 * other.e3 + e2 * other.e0 + e3 * other.e1 + e4 * other.e6 + e5 * other.e7 - e6 * other.e4 - e7 * other.e5;
            double a3 = e0 * other.e3 + e1 * other.e2 - e2 * other.e1 + e3 * other.e0 + e4 * other.e7 - e5 * other.e6 + e6 * other.e5 - e7 * other.e4;
            double a4 = e0 * other.e4 - e1 * other.e5 - e2 * other.e6 - e3 * other.e7 + e4 * other.e0 + e5 * other.e1 + e6 * other.e2 + e7 * other.e3;
            double a5 = e0 * other.e5 + e1 * other.e4 - e2 * other.e7 + e3 * other.e6 - e4 * other.e1 + e5 * other.e0 - e6 * other.e3 + e7 * other.e2;
            double a6 = e0 * other.e6 + e1 * other.e7 + e2 * other.e4 - e3 * other.e5 - e4 * other.e2 + e5 * other.e3 + e6 * other.e0 - e7 * other.e1;
            double a7 = e0 * other.e7 - e1 * other.e6 + e2 * other.e5 + e3 * other.e4 - e4 * other.e3 - e5 * other.e2 + e6 * other.e1 + e7 * other.e0;
            return new Octonion(a0, a1, a2, a3, a4, a5, a6, a7);
        }

        public float[] toEuler() {
            float yaw = (float) Math.atan2(2.0 * (e0 * e3 + e1 * e2 + e4 * e6 + e5 * e7),
                    1.0 - 2.0 * (e2 * e2 + e3 * e3 + e6 * e6 + e7 * e7));
            float pitch = (float) Math.asin(2.0 * (e0 * e2 - e3 * e1 + e4 * e7 - e5 * e6));
            return new float[]{(float) Math.toDegrees(yaw), (float) Math.toDegrees(pitch)};
        }
    }

    public static float perlinNoise(float x, float seed) {
        int i = (int) Math.floor(x);
        float f = x - i;
        float a0 = (float) (Math.sin(seed + i) * 0.5 + 0.5);
        float a1 = (float) (Math.sin(seed + i + 1) * 0.5 + 0.5);
        float t = f * f * (3.0f - 2.0f * f);
        return a0 + (a1 - a0) * t;
    }

    public static float[] bezierCurve(float[] start, float[] control1, float[] control2, float[] end, float t) {
        float t2 = t * t;
        float t3 = t2 * t;
        float mt = 1 - t;
        float mt2 = mt * mt;
        float mt3 = mt2 * mt;
        float x = mt3 * start[0] + 3 * mt2 * t * control1[0] + 3 * mt * t2 * control2[0] + t3 * end[0];
        float y = mt3 * start[1] + 3 * mt2 * t * control1[1] + 3 * mt * t2 * control2[1] + t3 * end[1];
        return new float[]{x, y};
    }

    public static float[] motionToRotation(EntityLivingBase target, float currentYaw, float currentPitch,
                                           float yawSpeedMin, float yawSpeedMax, float pitchSpeedMin, float pitchSpeedMax,
                                           float randomStrength, float predictionFactor, double range) {
        if (target == null || mc.thePlayer == null || !(target instanceof net.minecraft.entity.player.EntityPlayer) || target.equals(mc.thePlayer)) {
            return new float[]{currentYaw, currentPitch};
        }

        boolean targetChanged = lastTarget != target || lastTarget == null ||
                (lastTarget != null && lastTarget.getEntityId() != target.getEntityId());
        long currentTime = System.currentTimeMillis();
        boolean updateSeed = targetChanged || isKillAuraEnabled || (currentTime - lastSeedUpdate > 10);

        if (targetChanged) {
            notifyTargetSwitch();
            lastTargetRotations = new float[]{currentYaw, currentPitch};
        }

        updatePositionHistory(target);
        Vec3 predictedPos = predictPosition(target, predictionFactor, range);

        if (updateSeed || fixedRandomHeightOffset == 0) {
            seedRandom.setSeed(currentTime ^ (long) (mc.thePlayer.posX + mc.thePlayer.posZ + target.posX +
                    mc.thePlayer.ticksExisted + target.ticksExisted + target.motionX * 2000 + target.motionZ * 2000 +
                    (mc.thePlayer.isSprinting() ? 2000 : 0) + (mc.thePlayer.motionY > 0 ? 1000 : 0) +
                    targetCount * 350 + (currentTime - startTime) / 350));
            lastSeedUpdate = currentTime;
            lastTarget = target;
            fixedRandomHeightOffset = seedRandom.nextFloat();
            isKillAuraEnabled = false;
        }
        float seed = fixedRandomHeightOffset;

        float eyeHeight;
        AxisAlignedBB targetBB;
        try {
            eyeHeight = target.getEyeHeight();
            targetBB = target.getEntityBoundingBox();
            target.getArrowCountInEntity();
        } catch (Exception e) {
            eyeHeight = target instanceof net.minecraft.entity.player.EntityPlayer ? 1.62F : 1.0F;
            targetBB = new AxisAlignedBB(target.posX - 0.3, target.posY, target.posZ - 0.3,
                    target.posX + 0.3, target.posY + (target instanceof net.minecraft.entity.player.EntityPlayer ? 1.8F : 1.5F),
                    target.posZ + 0.3);
        }
        double bodyHeight = targetBB.maxY - targetBB.minY;
        float randomHeightOffset = (float) (seed * (bodyHeight - eyeHeight) * 0.85 + eyeHeight * 0.15);
        double aimY = predictedPos.yCoord + randomHeightOffset;

        Vec3 playerEyes = mc.thePlayer.getPositionEyes(1.0F);
        double deltaX = predictedPos.xCoord - playerEyes.xCoord;
        double deltaY = aimY - playerEyes.yCoord;
        double deltaZ = predictedPos.zCoord - playerEyes.zCoord;

        double distance = Math.sqrt(deltaX * deltaX + deltaZ * deltaZ);
        float targetYaw = (float) (Math.atan2(deltaZ, deltaX) * 180.0 / Math.PI - 90.0);
        float targetPitch = (float) (-(Math.atan2(deltaY, distance) * 180.0 / Math.PI));

        float yawDelta = MathHelper.wrapAngleTo180_float(targetYaw - currentYaw);
        float pitchDelta = MathHelper.clamp_float(targetPitch - currentPitch, -75.0F, 75.0F);
        targetYaw = currentYaw + MathHelper.clamp_float(yawDelta, -75.0F, 75.0F);
        targetPitch = currentPitch + MathHelper.clamp_float(pitchDelta, -75.0F, 75.0F);

        float sensitivity = mc.gameSettings.mouseSensitivity * 0.6F + 0.2F;
        float playerMotion = (float) Math.sqrt(mc.thePlayer.motionX * mc.thePlayer.motionX + mc.thePlayer.motionZ * mc.thePlayer.motionZ);
        float sensitivityFactor = sensitivity * sensitivity * sensitivity * 8.0F * (1.0F + seed * 0.08F);

        float[] boundingRotations = getBoundingBoxRotations(targetBB, playerEyes);
        targetYaw = clampToBoundingBox(targetYaw, boundingRotations[0], 4.0F);
        targetPitch = clampToBoundingBox(targetPitch, boundingRotations[1], 3.0F);

        float transitionFactor = targetChanged ? Math.min(1.0f, (currentTime - targetSwitchStartTime) / 200.0f) : 1.0f;
        float[] transitionRotations = targetChanged ? lastTargetRotations : new float[]{currentYaw, currentPitch};

        Octonion currentOct = eulerToOctonion(currentYaw, currentPitch, seed, playerMotion);
        Octonion targetOct = eulerToOctonion(targetYaw, targetPitch, seed, playerMotion);

        double targetSpeed = Math.sqrt(target.getDistance(target.lastTickPosX, target.lastTickPosY, target.lastTickPosZ));
        float jumpFactor = mc.thePlayer.motionY > 0 ? 0.2f : 0.0f;
        float switchFactor = targetCount > 2 ? 0.2f : (targetCount > 1 ? 0.15f : 0.0f);
        float speedMultiplier = (float) Math.min(1.0, range / (distance + 0.1)) *
                (float) (1.0 + targetSpeed * 0.07 + playerMotion * 0.04 + jumpFactor + switchFactor);
        float yawSpeed = yawSpeedMin + random.nextFloat() * (yawSpeedMax - yawSpeedMin) * speedMultiplier;
        float pitchSpeed = pitchSpeedMin + random.nextFloat() * (pitchSpeedMax - pitchSpeedMin) * speedMultiplier;
        float interpolationFactor = Math.min(yawSpeed, pitchSpeed) / 180.0F *
                (0.4f + (playerMotion > 0.4 ? 0.5f : (playerMotion < 0.1 ? 1.0f : 0.25f + playerMotion * 1.5f)));

        float delayFactor = Math.min(1.0F, (currentTime - lastRotationTime) /
                (4.0F + seed * 60.0F + perlinNoise(currentTime * 0.008f, seed) * 15.0f));
        lastRotationTime = currentTime;

        float pauseProbability = playerMotion < 0.1 ? 0.20f : 0.16f;
        if (random.nextFloat() < pauseProbability) {
            try {
                Thread.sleep(random.nextInt(12) + 1);
            } catch (InterruptedException ignored) {}
        }

        Octonion interpolatedOct = octonionSlerp(currentOct, targetOct, Math.min(interpolationFactor * delayFactor, 1.0F));
        float[] rotations = interpolatedOct.toEuler();

        float[] controlPoint1 = new float[]{
                transitionRotations[0] + gaussianRandom(0.7f),
                transitionRotations[1] + gaussianRandom(0.7f)
        };
        float[] controlPoint2 = new float[]{
                rotations[0] + gaussianRandom(0.7f),
                rotations[1] + gaussianRandom(0.7f)
        };
        rotations = bezierCurve(transitionRotations, controlPoint1, controlPoint2, rotations, transitionFactor);

        float dynamicRandomStrength = randomStrength * (float) Math.max(0.07, Math.min(1.0, distance / range)) *
                (1.0f + (currentTime - startTime) / 4000.0f);
        float sineOffset1 = (float) Math.sin(currentTime * (playerMotion > 0.4 ? 0.09 : 0.07) + seed) * 1.8f;
        float sineOffset2 = (float) Math.cos(currentTime * (playerMotion > 0.4 ? 0.035 : 0.025) + seed * 1.5) * 1.0f;
        float sineOffset3 = (float) Math.sin(currentTime * (playerMotion > 0.4 ? 0.015 : 0.01) + seed * 2.0) * 0.5f;
        float perlinOffset = perlinNoise(currentTime * 0.008f, seed) * 0.06f;
        rotations[0] += gaussianRandom(dynamicRandomStrength * (1.0F + seed * 0.35F)) + sineOffset1 + sineOffset2 + sineOffset3 + perlinOffset;
        rotations[1] += gaussianRandom(dynamicRandomStrength * (1.0F + seed * 0.35F)) + sineOffset1 * 0.5f + sineOffset2 * 0.3f + sineOffset3 * 0.2f + perlinOffset;

        if (random.nextFloat() < 0.35f) {
            rotations[0] += gaussianRandom(0.25f);
            rotations[1] += gaussianRandom(0.25f);
        }

        if (currentTime - lastResetTime > 1200) {
            rotations[0] += gaussianRandom(0.6f);
            rotations[1] += gaussianRandom(0.6f);
            lastResetTime = currentTime;
        }

        float angleIncrement = 0.0003F + seed * 0.3497F + perlinNoise(currentTime * 0.008f, seed) * 0.06f;
        if (currentTime - startTime > 40) {
            angleIncrement += random.nextFloat() * 0.015f - 0.0075f;
            startTime = currentTime;
        }
        if (currentTime - lastTargetSwitchTime > 400) {
            angleIncrement += random.nextFloat() * 0.06f - 0.03f;
        }
        if (currentTime - lastResetTime > 1200) {
            angleIncrement += random.nextFloat() * 0.2f - 0.1f;
        }
        rotations[0] = Math.round(rotations[0] / angleIncrement) * angleIncrement;
        rotations[1] = Math.round(rotations[1] / angleIncrement) * angleIncrement;

        rotations[0] = MathHelper.wrapAngleTo180_float(rotations[0]);
        rotations[1] = MathHelper.clamp_float(rotations[1], -90.0F, 90.0F);

        if (!mc.thePlayer.canEntityBeSeen(target)) {
            rotations[0] += gaussianRandom(0.25F);
            rotations[1] += gaussianRandom(0.25F);
        }

        lastTargetRotations = rotations;
        return rotations;
    }

    public static float gaussianRandom(float strength) {
        return (float) (random.nextGaussian() * strength * 0.7F);
    }

    private static void updatePositionHistory(Entity target) {
        positionHistory.add(new double[]{target.posX, target.posY, target.posZ});
        if (positionHistory.size() > 12) {
            positionHistory.remove(0);
        }
    }

    private static Vec3 predictPosition(EntityLivingBase target, float predictionFactor, double range) {
        double motionX = target.posX - target.lastTickPosX;
        double motionY = target.posY - target.lastTickPosY;
        double motionZ = target.posZ - target.lastTickPosZ;

        double accelX = 0, accelY = 0, accelZ = 0;
        if (positionHistory.size() >= 3) {
            double[] prev = positionHistory.get(positionHistory.size() - 2);
            double[] prevPrev = positionHistory.get(positionHistory.size() - 3);
            accelX = (target.posX - 2 * prev[0] + prevPrev[0]) * 0.5;
            accelY = (target.posY - 2 * prev[1] + prevPrev[1]) * 0.5;
            accelZ = (target.posZ - 2 * prev[2] + prevPrev[2]) * 0.5;
        }

        double friction = target.isInWater() ? 0.75 : (target.onGround ? 0.55 : 0.95);
        motionX *= friction;
        motionY *= target.onGround ? 0.0 : 0.97;
        motionZ *= friction;

        double distance;
        try {
            distance = mc.thePlayer.getDistanceToEntity(target);
        } catch (Exception e) {
            distance = Math.sqrt(Math.pow(target.posX - mc.thePlayer.posX, 2) +
                    Math.pow(target.posY - mc.thePlayer.posY, 2) +
                    Math.pow(target.posZ - mc.thePlayer.posZ, 2));
        }
        float adjustedPrediction = predictionFactor * (float) Math.min(1.0, range / (distance + 0.1));

        double predictedX = target.posX + motionX * adjustedPrediction + 0.5 * accelX * adjustedPrediction * adjustedPrediction;
        double predictedY = target.posY + motionY * adjustedPrediction + 0.5 * accelY * adjustedPrediction * adjustedPrediction;
        double predictedZ = target.posZ + motionZ * adjustedPrediction + 0.5 * accelZ * adjustedPrediction * adjustedPrediction;

        return new Vec3(predictedX, predictedY, predictedZ);
    }

    public static Octonion eulerToOctonion(float yaw, float pitch, float seed, float playerMotion) {
        double cy = Math.cos(Math.toRadians(yaw) * 0.5);
        double sy = Math.sin(Math.toRadians(yaw) * 0.5);
        double cp = Math.cos(Math.toRadians(pitch) * 0.5);
        double sp = Math.sin(Math.toRadians(pitch) * 0.5);
        double e4 = (playerMotion > 0.4 ? random.nextFloat() * 0.015 : 0) * seed;
        double e5 = (playerMotion > 0.4 ? random.nextFloat() * 0.015 : 0) * (1.0 - seed);
        double e6 = random.nextFloat() * 0.008 * seed;
        double e7 = random.nextFloat() * 0.008 * (1.0 - seed);
        return new Octonion(cy * cp, 0, sp * cy, sy * cp, e4, e5, e6, e7).normalize();
    }

    public static Octonion octonionSlerp(Octonion o1, Octonion o2, float t) {
        double dot = o1.e0 * o2.e0 + o1.e1 * o2.e1 + o1.e2 * o2.e2 + o1.e3 * o2.e3 +
                o1.e4 * o2.e4 + o1.e5 * o2.e5 + o1.e6 * o2.e6 + o1.e7 * o2.e7;
        if (dot < 0.0) {
            o2 = new Octonion(-o2.e0, -o2.e1, -o2.e2, -o2.e3, -o2.e4, -o2.e5, -o2.e6, -o2.e7);
            dot = -dot;
        }

        if (dot > 0.9995) {
            return new Octonion(
                    o1.e0 + t * (o2.e0 - o1.e0),
                    o1.e1 + t * (o2.e1 - o1.e1),
                    o1.e2 + t * (o2.e2 - o1.e2),
                    o1.e3 + t * (o2.e3 - o1.e3),
                    o1.e4 + t * (o2.e4 - o1.e4),
                    o1.e5 + t * (o2.e5 - o1.e5),
                    o1.e6 + t * (o2.e6 - o1.e6),
                    o1.e7 + t * (o2.e7 - o1.e7)
            ).normalize();
        }

        double theta = Math.acos(dot);
        double sinTheta = Math.sin(theta);
        float tAdjusted = (float) (t * t * (3 - 2 * t)) * (1.0f + random.nextFloat() * 0.05f);
        double scale1 = Math.sin((1 - tAdjusted) * theta) / sinTheta;
        double scale2 = Math.sin(tAdjusted * theta) / sinTheta;

        return new Octonion(
                scale1 * o1.e0 + scale2 * o2.e0,
                scale1 * o1.e1 + scale2 * o2.e1,
                scale1 * o1.e2 + scale2 * o2.e2,
                scale1 * o1.e3 + scale2 * o2.e3,
                scale1 * o1.e4 + scale2 * o2.e4,
                scale1 * o1.e5 + scale2 * o2.e5,
                scale1 * o1.e6 + scale2 * o2.e6,
                scale1 * o1.e7 + scale2 * o2.e7
        ).normalize();
    }

    private static float[] getBoundingBoxRotations(AxisAlignedBB bb, Vec3 playerEyes) {
        Vec3 center = new Vec3(
                (bb.minX + bb.maxX) / 2.0,
                (bb.minY + bb.maxY) / 2.0,
                (bb.minZ + bb.maxZ) / 2.0
        );
        double deltaX = center.xCoord - playerEyes.xCoord;
        double deltaY = center.yCoord - playerEyes.yCoord;
        double deltaZ = center.zCoord - playerEyes.zCoord;
        double distance = Math.sqrt(deltaX * deltaX + deltaZ * deltaZ);

        return new float[]{
                (float) (Math.atan2(deltaZ, deltaX) * 180.0 / Math.PI - 90.0),
                (float) (-(Math.atan2(deltaY, distance) * 180.0 / Math.PI))
        };
    }

    private static float clampToBoundingBox(float rotation, float centerRotation, float maxDeviation) {
        return MathHelper.clamp_float(rotation, centerRotation - maxDeviation, centerRotation + maxDeviation);
    }
}