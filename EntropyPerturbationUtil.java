package net.augustus.utils.combatModule.killAura;

import net.minecraft.entity.Entity;
import net.minecraft.entity.EntityLivingBase;
import net.minecraft.util.MathHelper;
import net.minecraft.util.Vec3;

import java.security.SecureRandom;

import static net.augustus.utils.interfaces.MC.mc;

public class EntropyPerturbationUtil {
    private final SecureRandom random = new SecureRandom();
    private float[] lastRotations = new float[]{0.0F, 0.0F};
    private double entropyFactor;
    private double smoothingFactor;
    private double minPerturbation;
    private double maxPerturbation;
    private long lastUpdateTime;

    public EntropyPerturbationUtil(double entropyFactor, double smoothingFactor, double minPerturbation, double maxPerturbation) {
        this.entropyFactor = entropyFactor;
        this.smoothingFactor = smoothingFactor;
        this.minPerturbation = minPerturbation;
        this.maxPerturbation = maxPerturbation;
        this.lastUpdateTime = System.currentTimeMillis();
    }

    public float[] calculateRotations(EntityLivingBase target, float currentYaw, float currentPitch, float yawSpeed, float pitchSpeed, double range, boolean throughWalls) {
        if (target == null) {
            return new float[]{currentYaw, currentPitch};
        }

        Vec3 playerEyes = mc.thePlayer.getPositionEyes(1.0F);
        Vec3 targetPos = getTargetPosition(target, range);

        double deltaX = targetPos.xCoord - playerEyes.xCoord;
        double deltaY = targetPos.yCoord - playerEyes.yCoord;
        double deltaZ = targetPos.zCoord - playerEyes.zCoord;

        double distance = Math.sqrt(deltaX * deltaX + deltaZ * deltaZ);
        float targetYaw = (float) (Math.atan2(deltaZ, deltaX) * 180.0 / Math.PI) - 90.0F;
        float targetPitch = (float) (-(Math.atan2(deltaY, distance) * 180.0 / Math.PI));

        float perturbedYaw = applyEntropyPerturbation(targetYaw, yawSpeed, currentYaw);
        float perturbedPitch = applyEntropyPerturbation(targetPitch, pitchSpeed, currentPitch);

        float smoothedYaw = interpolateRotation(currentYaw, perturbedYaw, (float) smoothingFactor);
        float smoothedPitch = interpolateRotation(currentPitch, perturbedPitch, (float) smoothingFactor);

        smoothedPitch = MathHelper.clamp_float(smoothedPitch, -90.0F, 90.0F);

        lastRotations = new float[]{smoothedYaw, smoothedPitch};

        return new float[]{smoothedYaw, smoothedPitch};
    }

    private Vec3 getTargetPosition(EntityLivingBase target, double range) {
        double predictFactor = 0.2;
        double motionX = (target.posX - target.lastTickPosX) * predictFactor;
        double motionY = (target.posY - target.lastTickPosY) * predictFactor;
        double motionZ = (target.posZ - target.lastTickPosZ) * predictFactor;

        double hitHeight = target.getEyeHeight() * (0.5 + random.nextDouble() * 0.5);
        return new Vec3(target.posX + motionX, target.posY + hitHeight + motionY, target.posZ + motionZ);
    }

    private float applyEntropyPerturbation(float targetAngle, float speed, float currentAngle) {
        float angleDiff = MathHelper.wrapAngleTo180_float(targetAngle - currentAngle);

        double perturbation = (random.nextDouble() * (maxPerturbation - minPerturbation) + minPerturbation) * entropyFactor;
        perturbation *= random.nextBoolean() ? 1 : -1;

        float perturbedAngle = targetAngle + (float) perturbation;

        float maxChange = speed / 20.0F;
        float newAngleDiff = MathHelper.wrapAngleTo180_float(perturbedAngle - currentAngle);
        newAngleDiff = MathHelper.clamp_float(newAngleDiff, -maxChange, maxChange);

        return currentAngle + newAngleDiff;
    }

    private float interpolateRotation(float current, float target, float factor) {
        float wrapped = MathHelper.wrapAngleTo180_float(target - current);
        return current + wrapped * factor;
    }

    public void reset() {
        lastRotations = new float[]{0.0F, 0.0F};
        lastUpdateTime = System.currentTimeMillis();
    }

    public float[] getLastRotations() {
        return lastRotations.clone();
    }
}