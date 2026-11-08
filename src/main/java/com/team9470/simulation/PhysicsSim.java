package com.team9470.simulation;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;

import com.team9470.FieldConstants;
import com.team9470.subsystems.intake.IntakeConstants;

public class PhysicsSim {

    // Geometry Classes
    private static class Triangle {
        final Translation3d p1, p2, p3;
        final Translation3d normal;

        public Triangle(Translation3d p1, Translation3d p2, Translation3d p3) {
            this.p1 = p1;
            this.p2 = p2;
            this.p3 = p3;
            // Compute Normal
            Translation3d u = p2.minus(p1);
            Translation3d v = p3.minus(p1);
            this.normal = crossProduct(u, v).div(crossProduct(u, v).getNorm());
        }
    }

    private static Translation3d crossProduct(Translation3d a, Translation3d b) {
        return new Translation3d(
                a.getY() * b.getZ() - a.getZ() * b.getY(),
                a.getZ() * b.getX() - a.getX() * b.getZ(),
                a.getX() * b.getY() - a.getY() * b.getX());
    }

    private static final List<Triangle> GOAL_MESH = new ArrayList<>();

    static {
        // Generate Hex Goal Mesh
        double topR = 0.6096;
        double botR = 0.3396;
        double topZ = 1.8288;
        double botZ = 1.4351;

        // 6 Segments
        for (int i = 0; i < 6; i++) {
            double theta1 = Math.toRadians(60 * i);
            double theta2 = Math.toRadians(60 * (i + 1));

            Translation3d t1 = new Translation3d(topR * Math.cos(theta1), topR * Math.sin(theta1), topZ);
            Translation3d t2 = new Translation3d(topR * Math.cos(theta2), topR * Math.sin(theta2), topZ);
            Translation3d b1 = new Translation3d(botR * Math.cos(theta1), botR * Math.sin(theta1), botZ);
            Translation3d b2 = new Translation3d(botR * Math.cos(theta2), botR * Math.sin(theta2), botZ);

            GOAL_MESH.add(new Triangle(t1, b1, t2));
            GOAL_MESH.add(new Triangle(t2, b1, b2));
        }
    }

    /**
     * Main Simulation Loop
     * Integrates all projectiles and resolves collisions in lock-step substeps.
     * This ensures high-fidelity interaction (no tunneling, accurate momentum
     * transfer).
     */
    public static void simulate(double totalDt, List<Projectile> projectiles, Translation3d goalCenter,
            edu.wpi.first.math.geometry.Pose2d robotPose, edu.wpi.first.math.kinematics.ChassisSpeeds robotSpeeds,
            double intakeAngle, boolean intakeActive) {
        int substeps = 8;
        double dt = totalDt / substeps;

        for (int step = 0; step < substeps; step++) {
            // 1. Integrate All
            for (Projectile p : projectiles) {
                p.integrateSubstep(dt, goalCenter, robotPose, robotSpeeds, intakeAngle, intakeActive);
            }

            // 2. Resolve Collisions (Every substep!)
            if (projectiles.size() > 1) {
                resolveCollisionsEfficient(projectiles);
            }
        }
    }

    public static class Projectile {
        public Translation3d position;
        public Translation3d velocity;
        public boolean isStored = false;
        public boolean isScored = false;
        public Translation3d angularVelocity; // rad/s

        // Physics Constants
        private static final double GRAVITY = 9.81; // m/s^2
        private static final double AIR_DENSITY = 1.225; // kg/m^3
        private static final double DRAG_COEFF = 0.5;
        private static final double MASS = 0.2268; // 0.5 lbs
        public static final double RADIUS = 0.07493; // 2.95 inches (5.9in diam)
        private static final double AREA = Math.PI * RADIUS * RADIUS;

        private static final double PLATE_THICKNESS = Units.inchesToMeters(0.125);
        private static final double COLLISION_RADIUS = RADIUS + (PLATE_THICKNESS / 2.0);

        public Projectile(Translation3d position, Translation3d velocity) {
            this(position, velocity, new Translation3d());
        }

        public Projectile(Translation3d position, Translation3d velocity, Translation3d angularVelocity) {
            this.position = position;
            this.velocity = velocity;
            this.angularVelocity = angularVelocity;
        }

        public void integrateSubstep(double dt, Translation3d goalCenter, edu.wpi.first.math.geometry.Pose2d robotPose,
                edu.wpi.first.math.kinematics.ChassisSpeeds robotSpeeds,
                double intakeAngle, boolean intakeActive) {

            // Forces
            VelocityForces forces = calculateForces();

            // Intake Force
            // Intake Force - REMOVED (Now using Impulse Collision)

            // Symplectic Integration
            Translation3d accel = forces.fNet.div(MASS);
            velocity = velocity.plus(accel.times(dt));
            position = position.plus(velocity.times(dt));

            // 1. Goal Collision (Mesh)
            if (resolveGoalCollision(goalCenter)) {
                isScored = true;
            }

            if (!isStored) {
                // 2. Robot Collision (External)
                resolveRobotCollision(robotPose, robotSpeeds);
                // 3. Field Collision
                resolveFieldCollision(dt);
                // 4. Check Capture (Only if intake active)
                // 4. Check Intake Collision (Only if intake active)
                if (intakeActive) {
                    resolveIntakeCollision(robotPose, intakeAngle);
                    checkHopperCapture(robotPose);
                }
            } else {
                // 5. Hopper Constraint (Internal)
                resolveHopperConstraint(robotPose);
            }
        }

        private void resolveIntakeCollision(edu.wpi.first.math.geometry.Pose2d robotPose, double intakeAngle) {
            // Intake Geometry
            double pX = 0.35;
            double pZ = 0.2;
            double armLen = IntakeConstants.kIntakeLength;
            double rollerRadius = 0.05;
            double rollerWidth = 0.6; // Intake width overlapping the bumper

            // 1. Calculate Roller Center in Robot Frame (XZ Plane)
            double rRelX = pX + armLen * Math.cos(intakeAngle);
            double rRelZ = pZ + armLen * Math.sin(intakeAngle);

            // 2. Convert Ball Position to Robot Frame
            double dx = position.getX() - robotPose.getX();
            double dy = position.getY() - robotPose.getY();
            double cos = robotPose.getRotation().getCos();
            double sin = robotPose.getRotation().getSin();

            // localX = Forward, localY = Left
            double localX = dx * cos + dy * sin;
            double localY = -dx * sin + dy * cos;
            double localZ = position.getZ();

            // 3. Check Width (Y-axis)
            if (Math.abs(localY) > rollerWidth / 2.0) {
                return; // Missed sideways
            }

            // 4. Check Distance in XZ Plane (Roller Cross-section)
            double distXZ = Math.hypot(localX - rRelX, localZ - rRelZ);
            double collisionDist = RADIUS + rollerRadius;

            if (distXZ < collisionDist) {
                // CONTACT!
                // Kick Velocity: 8 m/s IN (-X), 4 m/s UP (+Z) relative to robot
                Translation3d robotFwd = new Translation3d(cos, sin, 0);
                Translation3d robotUp = new Translation3d(0, 0, 1);

                // Kick Velocity: 10 m/s Impulse
                // Direction: Into Robot (-Forward) + Up
                Translation3d kickDir = robotFwd.times(-1.0).plus(robotUp.times(0.5));
                kickDir = kickDir.div(kickDir.getNorm());

                double kickSpeed = 10.0;

                // Set Velocity
                velocity = kickDir.times(kickSpeed);
            }
        }

        private void checkHopperCapture(edu.wpi.first.math.geometry.Pose2d robotPose) {
            double dx = position.getX() - robotPose.getX();
            double dy = position.getY() - robotPose.getY();
            double cos = robotPose.getRotation().getCos();
            double sin = robotPose.getRotation().getSin();
            double localX = dx * cos + dy * sin;
            double localZ = position.getZ();

            // Capture Volume
            // Lowered floor to 0.1 to catch balls kicked by intake before they rise too
            // high
            if (localZ > 0.1 && localZ < 0.6 && localX > 0.0 && localX < 0.5) {
                isStored = true;
                // NO DAMPING - Let it rattle!
            }
        }

        private void resolveHopperConstraint(edu.wpi.first.math.geometry.Pose2d robotPose) {
            double hSize = Units.inchesToMeters(27.0);
            double half = hSize / 2.0;

            double minX = -half;
            double maxX = half;
            double minY = -half;
            double maxY = half;
            double minZ = 0.15;
            double maxZ = minZ + hSize;

            double dx = position.getX() - robotPose.getX();
            double dy = position.getY() - robotPose.getY();
            double cos = robotPose.getRotation().getCos();
            double sin = robotPose.getRotation().getSin();
            double localX = dx * cos + dy * sin;
            double localY = -dx * sin + dy * cos;
            double localZ = position.getZ();

            boolean clamped = false;

            if (localX < minX + RADIUS) {
                localX = minX + RADIUS;
                clamped = true;
            }
            if (localX > maxX - RADIUS) {
                localX = maxX - RADIUS;
                clamped = true;
            }
            if (localY < minY + RADIUS) {
                localY = minY + RADIUS;
                clamped = true;
            }
            if (localY > maxY - RADIUS) {
                localY = maxY - RADIUS;
                clamped = true;
            }
            if (localZ < minZ + RADIUS) {
                localZ = minZ + RADIUS;
                clamped = true;
            }
            if (localZ > maxZ - RADIUS) {
                localZ = maxZ - RADIUS;
                clamped = true;
            }

            if (clamped) {
                velocity = velocity.times(0.5);
                double worldX = localX * cos - localY * sin;
                double worldY = localX * sin + localY * cos;
                position = new Translation3d(
                        robotPose.getX() + worldX,
                        robotPose.getY() + worldY,
                        localZ);
            }
        }

        private static class VelocityForces {
            Translation3d fNet;
        }

        private VelocityForces calculateForces() {
            Translation3d Fg = new Translation3d(0, 0, -MASS * GRAVITY);

            double speed = velocity.getNorm();
            Translation3d Fd = new Translation3d();
            if (speed > 1e-6) {
                double dragMag = 0.5 * AIR_DENSITY * AREA * DRAG_COEFF * speed * speed;
                Fd = velocity.div(speed).times(-dragMag);
            }

            Translation3d Fm = new Translation3d();
            if (angularVelocity.getNorm() > 1e-6 && speed > 1e-6) {
                Translation3d wCrossV = crossProduct(angularVelocity, velocity);
                double liftFactor = 0.5 * AIR_DENSITY * AREA * RADIUS;
                Fm = wCrossV.times(liftFactor);
            }

            VelocityForces v = new VelocityForces();
            v.fNet = Fg.plus(Fd).plus(Fm);
            return v;
        }

        private boolean resolveGoalCollision(Translation3d goalCenter) {
            Translation3d localPos = new Translation3d(
                    position.getX() - goalCenter.getX(),
                    position.getY() - goalCenter.getY(),
                    position.getZ());

            if (Math.abs(localPos.getZ() - 1.5) > 0.7 || Math.hypot(localPos.getX(), localPos.getY()) > 1.0) {
                return false;
            }

            for (Triangle tri : GOAL_MESH) {
                Translation3d closest = closestPointOnTriangle(tri, localPos);
                Translation3d diff = localPos.minus(closest);
                double dist = diff.getNorm();

                if (dist < COLLISION_RADIUS) {
                    Translation3d normal;
                    if (dist < 1e-6)
                        normal = tri.normal;
                    else
                        normal = diff.div(dist);

                    double penetration = COLLISION_RADIUS - dist;
                    position = position.plus(normal.times(penetration));
                    localPos = localPos.plus(normal.times(penetration));

                    double vDotN = velocity.getX() * normal.getX() +
                            velocity.getY() * normal.getY() +
                            velocity.getZ() * normal.getZ();

                    if (vDotN < 0) {
                        double restitution = 0.5;
                        velocity = velocity.minus(normal.times((1 + restitution) * vDotN));
                    }
                }
            }

            double botZ = 1.4351;
            if (position.getZ() < botZ && position.getZ() > botZ - 0.5) {
                Translation3d rel = position.minus(goalCenter);
                if (Math.hypot(rel.getX(), rel.getY()) < 0.33) {
                    return true;
                }
            }
            return false;
        }

        private void resolveRobotCollision(edu.wpi.first.math.geometry.Pose2d robotPose,
                edu.wpi.first.math.kinematics.ChassisSpeeds robotSpeeds) {
            double rLen = Units.inchesToMeters(33.0);
            double rWidth = Units.inchesToMeters(33.0);
            double rHeight = Units.inchesToMeters(5.0);

            double dx = position.getX() - robotPose.getX();
            double dy = position.getY() - robotPose.getY();
            double cos = robotPose.getRotation().getCos();
            double sin = robotPose.getRotation().getSin();
            double localX = dx * cos + dy * sin;
            double localY = -dx * sin + dy * cos;
            double localZ = position.getZ();

            double cx = Math.max(-rLen / 2, Math.min(rLen / 2, localX));
            double cy = Math.max(-rWidth / 2, Math.min(rWidth / 2, localY));
            double cz = Math.max(0, Math.min(rHeight, localZ));

            double distSq = Math.pow(localX - cx, 2) + Math.pow(localY - cy, 2) + Math.pow(localZ - cz, 2);

            if (distSq < RADIUS * RADIUS) {
                double dist = Math.sqrt(distSq);

                double nx, ny, nz;
                if (localZ < rHeight * 1.5) {
                    nz = 0;
                    double horizontalDist = Math.hypot(localX - cx, localY - cy);
                    if (horizontalDist < 1e-6) {
                        nx = (Math.abs(localX) > Math.abs(localY)) ? Math.signum(localX) : 0;
                        ny = (Math.abs(localY) >= Math.abs(localX)) ? Math.signum(localY) : 0;
                    } else {
                        nx = (localX - cx) / horizontalDist;
                        ny = (localY - cy) / horizontalDist;
                    }
                } else {
                    if (dist < 1e-6) {
                        nx = 0;
                        ny = 0;
                        nz = 1;
                    } else {
                        nx = (localX - cx) / dist;
                        ny = (localY - cy) / dist;
                        nz = (localZ - cz) / dist;
                    }
                }

                double worldNx = nx * cos - ny * sin;
                double worldNy = nx * sin + ny * cos;
                double worldNz = nz;
                Translation3d normal = new Translation3d(worldNx, worldNy, worldNz);

                double penetration = RADIUS - dist;
                position = position.plus(normal.times(penetration));

                // ROBOT VELOCITY AT IMPACT POINT
                // V_point = V_com + Omega x R
                double vxRobot = robotSpeeds.vxMetersPerSecond;
                double vyRobot = robotSpeeds.vyMetersPerSecond;
                double omega = robotSpeeds.omegaRadiansPerSecond;

                // r is vector from robot center to collision point (dx, dy)
                // v_tangential = omega * r (perpendicular)
                // v_x_rot = -omega * dy
                // v_y_rot = omega * dx
                double vxPoint = vxRobot - omega * dy;
                double vyPoint = vyRobot + omega * dx;

                Translation3d vRobot = new Translation3d(vxPoint, vyPoint, 0);

                // Relative Velocity
                Translation3d vRel = velocity.minus(vRobot);

                double vDotN = vRel.getX() * normal.getX() +
                        vRel.getY() * normal.getY() +
                        vRel.getZ() * normal.getZ();

                if (vDotN < 0) {
                    double restitution = 0.5; // Bumpier collision
                    Translation3d impulse = normal.times(-(1 + restitution) * vDotN);

                    // Apply impulse to relative velocity
                    vRel = vRel.plus(impulse);

                    // Convert back to world velocity
                    velocity = vRel.plus(vRobot);
                }
            }
        }

        private void resolveFieldCollision(double dt) {
            double fieldLen = FieldConstants.fieldLength;
            double fieldWidth = FieldConstants.fieldWidth;
            double ballRadius = RADIUS;
            double restitution = 0.4;

            // Floor Interaction
            if (position.getZ() <= ballRadius) {
                // 1. Normal Force / Bounce
                if (velocity.getZ() < 0) {
                    velocity = new Translation3d(velocity.getX(), velocity.getY(), -velocity.getZ() * restitution);
                }
                position = new Translation3d(position.getX(), position.getY(), ballRadius); // Hard Constraint

                // 2. Friction & Rotation (Rolling Physics)
                // Tangential Velocity at Contact Point = V_cm + Omega x R
                // Contact point is (0,0, -R) relative to center
                Translation3d r = new Translation3d(0, 0, -ballRadius);
                Translation3d omegaCrossR = crossProduct(angularVelocity, r);
                Translation3d vContact = velocity.plus(omegaCrossR);

                // We only care about XY plane friction for the floor
                Translation3d vSlip = new Translation3d(vContact.getX(), vContact.getY(), 0.0);
                double slipSpeed = vSlip.getNorm();

                double mu = 0.4; // Kinetic Friction
                double normalForce = MASS * GRAVITY;

                if (slipSpeed > 0.05) { // Significant slip -> Apply Kinetic Friction
                    // Friction Force opposes slip
                    Translation3d frictionDir = vSlip.div(slipSpeed).times(-1.0);
                    Translation3d frictionForce = frictionDir.times(mu * normalForce);

                    // Linear Impulse: dV = F * dt / m
                    Translation3d deltaV = frictionForce.times(dt / MASS);

                    // Angular Impulse: dw = (r x F) * dt / I
                    // Solid Sphere I = 2/5 * m * r^2
                    double I = 0.4 * MASS * ballRadius * ballRadius;
                    Translation3d torque = crossProduct(r, frictionForce);
                    Translation3d deltaW = torque.times(dt / I);

                    velocity = velocity.plus(deltaV);
                    angularVelocity = angularVelocity.plus(deltaW);

                } else {
                    // Rolling State (vContact ~ 0)
                    // Apply Rolling Resistance (Drag)
                    // This mimics deformation energy loss, prevents permanent perpetual motion
                    velocity = velocity.times(0.998);
                    angularVelocity = angularVelocity.times(0.998);
                }
            }

            // Walls (Simple AABB)
            if (position.getX() < ballRadius) {
                if (velocity.getX() < 0)
                    velocity = new Translation3d(-velocity.getX() * restitution, velocity.getY(), velocity.getZ());
                position = new Translation3d(ballRadius, position.getY(), position.getZ());
            }
            if (position.getX() > fieldLen - ballRadius) {
                if (velocity.getX() > 0)
                    velocity = new Translation3d(-velocity.getX() * restitution, velocity.getY(), velocity.getZ());
                position = new Translation3d(fieldLen - ballRadius, position.getY(), position.getZ());
            }
            if (position.getY() < ballRadius) {
                if (velocity.getY() < 0)
                    velocity = new Translation3d(velocity.getX(), -velocity.getY() * restitution, velocity.getZ());
                position = new Translation3d(position.getX(), ballRadius, position.getZ());
            }
            if (position.getY() > fieldWidth - ballRadius) {
                if (velocity.getY() > 0)
                    velocity = new Translation3d(velocity.getX(), -velocity.getY() * restitution, velocity.getZ());
                position = new Translation3d(position.getX(), fieldWidth - ballRadius, position.getZ());
            }
        }
    }

    // ... Static Helpers ...
    private static double dotProduct(Translation3d a, Translation3d b) {
        return a.getX() * b.getX() + a.getY() * b.getY() + a.getZ() * b.getZ();
    }

    private static Translation3d closestPointOnTriangle(Triangle tri, Translation3d p) {
        Translation3d a = tri.p1;
        Translation3d b = tri.p2;
        Translation3d c = tri.p3;

        Translation3d ab = b.minus(a);
        Translation3d ac = c.minus(a);
        Translation3d ap = p.minus(a);

        double d1 = dotProduct(ab, ap);
        double d2 = dotProduct(ac, ap);
        if (d1 <= 0.0 && d2 <= 0.0)
            return a;

        Translation3d bp = p.minus(b);
        double d3 = dotProduct(ab, bp);
        double d4 = dotProduct(ac, bp);
        if (d3 >= 0.0 && d4 <= d3)
            return b;

        double vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
            double v = d1 / (d1 - d3);
            return a.plus(ab.times(v));
        }

        Translation3d cp = p.minus(c);
        double d5 = dotProduct(ab, cp);
        double d6 = dotProduct(ac, cp);
        if (d6 >= 0.0 && d5 <= d6)
            return c;

        double vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
            double w = d2 / (d2 - d6);
            return a.plus(ac.times(w));
        }

        double va = d3 * d6 - d5 * d4;
        if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
            double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return b.plus(c.minus(b).times(w));
        }

        double denom = 1.0 / (va + vb + vc);
        double v = vb * denom;
        double w = vc * denom;
        return a.plus(ab.times(v)).plus(ac.times(w));
    }

    public static void resolveCollision(Projectile a, Projectile b) {
        double radius = Projectile.RADIUS;
        double dist = a.position.getDistance(b.position);

        if (dist < radius * 2) {
            Translation3d n = a.position.minus(b.position).div(dist);
            Translation3d vRel = a.velocity.minus(b.velocity);
            double velAlongNormal = dotProduct(vRel, n);

            if (velAlongNormal > 0)
                return;

            double e = 0.5;
            double j = -(1 + e) * velAlongNormal;
            j /= 2.0;

            Translation3d impulse = n.times(j);
            a.velocity = a.velocity.plus(impulse);
            b.velocity = b.velocity.minus(impulse);

            double percent = 0.8;
            double slop = 0.001;
            double penetration = (radius * 2) - dist;
            if (penetration > slop) {
                Translation3d correction = n.times(penetration / 2.0 * percent);
                a.position = a.position.plus(correction);
                b.position = b.position.minus(correction);
            }
        }
    }

    private static long getGridKey(Translation3d pos, double cellSize) {
        int x = (int) Math.floor(pos.getX() / cellSize);
        int y = (int) Math.floor(pos.getY() / cellSize);
        return (((long) y) << 32) | (x & 0xFFFFFFFFL);
    }

    public static void resolveCollisionsEfficient(List<Projectile> projectiles) {
        double cellSize = Projectile.RADIUS * 4.0;
        java.util.Map<Long, List<Projectile>> grid = new java.util.HashMap<>();

        for (Projectile p : projectiles) {
            long key = getGridKey(p.position, cellSize);
            grid.computeIfAbsent(key, k -> new ArrayList<>()).add(p);
        }

        for (Projectile p : projectiles) {
            long MyKey = getGridKey(p.position, cellSize);
            int cx = (int) (MyKey);
            int cy = (int) (MyKey >> 32);

            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    long nKey = (((long) (cy + dy)) << 32) | ((cx + dx) & 0xFFFFFFFFL);
                    List<Projectile> cell = grid.get(nKey);

                    if (cell != null) {
                        for (Projectile other : cell) {
                            if (p != other && System.identityHashCode(p) < System.identityHashCode(other)) {
                                resolveCollision(p, other);
                            }
                        }
                    }
                }
            }
        }
    }
}
