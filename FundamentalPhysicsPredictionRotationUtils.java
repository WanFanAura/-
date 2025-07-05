package net.augustus.utils.killAura;

import net.augustus.utils.legitReach.RandomUtil;
import net.minecraft.client.Minecraft;
import net.minecraft.entity.EntityLivingBase;
import net.minecraft.util.MathHelper;
import net.minecraft.util.Vec3;
import org.lwjgl.input.Mouse;
import java.security.SecureRandom;

public class FundamentalPhysicsPredictionRotationUtils {
    private static final Minecraft mc = Minecraft.getMinecraft();

    /**
     * @param target 目标实体。
     * @param currentRotations 当前 [yaw, pitch] 旋转角度。
     * @param yawSpeedMin 最小 yaw 速度。
     * @param yawSpeedMax 最大 yaw 速度。
     * @param pitchSpeedMin 最小 pitch 速度。
     * @param pitchSpeedMax 最大 pitch 速度。
     * @param randomStrength 随机化强度。
     * @param interpolation 是否启用插值平滑。
     * @param randomize 是否对目标位置应用随机化。
     * @param scaffoldToggled 是否启用Scaffold模块。
     * @return 新的 [yaw, pitch] 旋转角度。
     */
    public static float[] rotateMotion(EntityLivingBase target, float[] currentRotations, float yawSpeedMin, float yawSpeedMax, float pitchSpeedMin, float pitchSpeedMax, double randomStrength, boolean interpolation, boolean randomize, boolean scaffoldToggled) {
        if (target == null || mc.thePlayer == null) {
            return currentRotations;
        }

        SecureRandom secureRandom = new SecureRandom();
        float deltaYaw = RandomUtil.nextFloat(yawSpeedMin, yawSpeedMax);
        float deltaPitch = RandomUtil.nextFloat(pitchSpeedMin, pitchSpeedMax);

        Vec3 playerEyes = mc.thePlayer.getPositionEyes(1.0F);

        Vec3 targetPos = new Vec3(target.posX, target.posY + target.getEyeHeight(), target.posZ);

        if (randomize) {
            targetPos = new Vec3(
                    targetPos.xCoord + (secureRandom.nextDouble() * 2 - 1) * randomStrength,
                    targetPos.yCoord + (secureRandom.nextDouble() * 2 - 1) * randomStrength,
                    targetPos.zCoord + (secureRandom.nextDouble() * 2 - 1) * randomStrength
            );
        }

        float[] targetRotations = calculateRotations(playerEyes, targetPos);

        if (interpolation) {
            float distance = (float) mc.thePlayer.getDistanceToEntity(target);
            float[] smoothedRotations = applyBezierSmoothing(currentRotations, targetRotations, deltaYaw, deltaPitch, distance, target);
            targetRotations[0] = smoothedRotations[0];
            targetRotations[1] = smoothedRotations[1];
        }

        targetRotations[1] = MathHelper.clamp_float(targetRotations[1], -90.0F, 90.0F);

        if (!scaffoldToggled && mc.thePlayer != null) {
            applyRotationWithMouseInput(targetRotations[0], targetRotations[1], mc.thePlayer.rotationYaw, mc.thePlayer.rotationPitch);
        }

        return targetRotations;
    }

    /**
     * @param targetYaw 目标 yaw。
     * @param targetPitch 目标 pitch。
     * @param currentYaw 当前 yaw。
     * @param currentPitch 当前 pitch。
     */
    public static void applyRotationWithMouseInput(float targetYaw, float targetPitch, float currentYaw, float currentPitch) {
        if (mc.thePlayer == null) return;

        float deltaYaw = MathHelper.wrapAngleTo180_float(targetYaw - currentYaw);
        float deltaPitch = targetPitch - currentPitch;

        float sensitivity = mc.gameSettings.mouseSensitivity;
        float scaledSensitivity = sensitivity * 0.6F + 0.2F;
        float sensitivityFactor = scaledSensitivity * scaledSensitivity * scaledSensitivity * 8.0F;

        int dx = (int) (deltaYaw / sensitivityFactor);
        int dy = (int) (-deltaPitch / sensitivityFactor);

        Mouse.getDX();
        Mouse.getDY();
        mc.thePlayer.rotationYaw += dx * sensitivityFactor;
        mc.thePlayer.rotationPitch += dy * sensitivityFactor;

        mc.thePlayer.rotationPitch = MathHelper.clamp_float(mc.thePlayer.rotationPitch, -90.0F, 90.0F);
    }

    /**
     * @param playerEyes 玩家眼睛位置。
     * @param targetPos 目标位置。
     * @return [yaw, pitch] 旋转角度。
     */
    private static float[] calculateRotations(Vec3 playerEyes, Vec3 targetPos) {
        double deltaX = targetPos.xCoord - playerEyes.xCoord;
        double deltaY = targetPos.yCoord - playerEyes.yCoord;
        double deltaZ = targetPos.zCoord - playerEyes.zCoord;

        double distanceXZ = Math.sqrt(deltaX * deltaX + deltaZ * deltaZ);
        float yaw = (float) Math.toDegrees(Math.atan2(deltaZ, deltaX)) - 90.0F;
        float pitch = (float) -Math.toDegrees(Math.atan2(deltaY, distanceXZ));

        yaw = MathHelper.wrapAngleTo180_float(yaw);
        return new float[]{yaw, pitch};
    }

    /**
     * @param currentRotations 当前旋转角度 [yaw, pitch]。
     * @param targetRotations 目标旋转角度 [yaw, pitch]。
     * @param deltaYaw yaw 速度。
     * @param deltaPitch pitch 速度。
     * @param distance 到目标的距离。
     * @param target 目标实体。
     * @return 平滑后的 [yaw, pitch] 旋转角度。
     */
    public static float[] applyBezierSmoothing(float[] currentRotations, float[] targetRotations, float deltaYaw, float deltaPitch, float distance, EntityLivingBase target) {
        double speed = Math.sqrt(target.motionX * target.motionX + target.motionY * target.motionY + target.motionZ * target.motionZ);
        float smoothFactor = MathHelper.clamp_float((float) (1.0 - distance / 15.0 + speed * 0.3), 0.3F, 0.9F);

        float deltaYawDiff = MathHelper.wrapAngleTo180_float(targetRotations[0] - currentRotations[0]);
        float deltaPitchDiff = targetRotations[1] - currentRotations[1];
        float controlYaw1 = currentRotations[0] + deltaYawDiff * 0.25F;
        float controlYaw2 = currentRotations[0] + deltaYawDiff * 0.75F;
        float controlPitch1 = currentRotations[1] + deltaPitchDiff * 0.25F;
        float controlPitch2 = currentRotations[1] + deltaPitchDiff * 0.75F;

        float t = smoothFactor;
        float invT = 1.0F - t;
        float yaw = invT * invT * invT * currentRotations[0] +
                3 * invT * invT * t * controlYaw1 +
                3 * invT * t * t * controlYaw2 +
                t * t * t * targetRotations[0];
        float pitch = invT * invT * invT * currentRotations[1] +
                3 * invT * invT * t * controlPitch1 +
                3 * invT * t * t * controlPitch2 +
                t * t * t * targetRotations[1];

        yaw = currentRotations[0] + MathHelper.clamp_float(MathHelper.wrapAngleTo180_float(yaw - currentRotations[0]), -deltaYaw, deltaYaw);
        pitch = currentRotations[1] + MathHelper.clamp_float(pitch - currentRotations[1], -deltaPitch, deltaPitch);

        SecureRandom random = new SecureRandom();
        yaw += random.nextGaussian() * 0.02F;
        pitch += random.nextGaussian() * 0.02F;

        return new float[]{yaw, pitch};
    }
}