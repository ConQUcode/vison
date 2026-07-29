% Temporary 3-DOF arm model plot for the current fixed test point.
% Coordinate frame:
%   +X: vehicle front
%   +Y: vehicle left
%   +Z: upward
% Origin is the GM6020 base yaw axis center.
%
% Firmware model used here:
%   shoulder axis = (0, -4, 94) mm
%   L1 = 150 mm, L2 = 141 mm
%   target wrist center = (200, 0, 0) mm
%   q3 is the signed relative elbow angle: q3=0 is same direction as L1,
%   q3=-180 is opposite direction to L1.
%   Measured mechanical elbow angle is approximately 49..143 deg, so
%   firmware q3 = mechanical angle - 180 deg = -131..-37 deg.

clear;
clc;
close all;

base = [0, 0, 0];
shoulder = [0, -4, 94];
L1 = 150;
L2 = 141;
target = [200, 0, 0];

% Solve the same geometry used by firmware IK.
rho = hypot(target(1), target(2));
localXMag = sqrt(max(0, rho^2 - shoulder(2)^2));
targetAzimuth = atan2(target(2), target(1));
zPlanar = target(3) - shoulder(3);

% Use positive local-X and negative q3. This is the branch inside the
% current q3 soft limit [-126, -42] deg.
localX = localXMag;
signedRadius = localX - shoulder(1);
q1 = targetAzimuth - atan2(shoulder(2), localX);

cosQ3 = (signedRadius^2 + zPlanar^2 - L1^2 - L2^2) / (2 * L1 * L2);
cosQ3 = max(-1, min(1, cosQ3));
q3 = -acos(cosQ3);
q2 = atan2(zPlanar, signedRadius) - ...
     atan2(L2 * sin(q3), L1 + L2 * cos(q3));

qDeg = rad2deg([q1, q2, q3]);
smallLinkPitchDeg = qDeg(2) + qDeg(3);
elbowReferenceDeg = -131;
elbowOppositeDeg = -37;
elbowOpenFromReferenceDeg = qDeg(3) - elbowReferenceDeg;
elbowToOppositeLimitDeg = elbowOppositeDeg - qDeg(3);

% Joint positions in the arm local vertical plane.
shoulderLocal = shoulder;
elbowLocal = [shoulder(1) + L1 * cos(q2), ...
              shoulder(2), ...
              shoulder(3) + L1 * sin(q2)];
wristLocal = [shoulder(1) + L1 * cos(q2) + L2 * cos(q2 + q3), ...
              shoulder(2), ...
              shoulder(3) + L1 * sin(q2) + L2 * sin(q2 + q3)];

% Rotate local plane around Z by q1 to get vehicle-frame coordinates.
Rz = [cos(q1), -sin(q1), 0; ...
      sin(q1),  cos(q1), 0; ...
      0,        0,       1];
shoulderWorld = (Rz * shoulderLocal.').';
elbowWorld = (Rz * elbowLocal.').';
wristWorld = (Rz * wristLocal.').';

fkError = norm(wristWorld - target);
horizontalDistance = hypot(target(1), target(2));
shoulderToTargetPlanar = hypot(signedRadius, zPlanar);

fprintf('Target wrist center: [%.3f, %.3f, %.3f] mm\n', target);
fprintf('Solved q: q1=%.3f deg, q2=%.3f deg, q3=%.3f deg\n', qDeg);
fprintf('Small link absolute pitch q2+q3 = %.3f deg\n', smallLinkPitchDeg);
fprintf('Elbow opening/travel from reference = %.3f deg\n', ...
        elbowOpenFromReferenceDeg);
fprintf('Elbow remaining to opposite limit = %.3f deg\n', ...
        elbowToOppositeLimitDeg);
fprintf('Shoulder to target planar distance = %.3f mm\n', ...
        shoulderToTargetPlanar);
fprintf('FK wrist center: [%.3f, %.3f, %.3f] mm\n', wristWorld);
fprintf('FK error = %.6f mm\n', fkError);

figure('Name', '3DOF Arm Target Debug', 'Color', 'w', ...
       'Position', [80, 80, 1320, 720]);

% ---------------- 3D vehicle-frame view ----------------
subplot(1, 2, 1);
hold on;
grid on;
axis equal;
xlabel('+X vehicle front (mm)');
ylabel('+Y vehicle left (mm)');
zlabel('+Z upward (mm)');
title('3D vehicle-frame view');

plot3([base(1), shoulderWorld(1)], ...
      [base(2), shoulderWorld(2)], ...
      [base(3), shoulderWorld(3)], 'k--', 'LineWidth', 1.4);
plot3([shoulderWorld(1), elbowWorld(1)], ...
      [shoulderWorld(2), elbowWorld(2)], ...
      [shoulderWorld(3), elbowWorld(3)], 'b-', 'LineWidth', 4);
plot3([elbowWorld(1), wristWorld(1)], ...
      [elbowWorld(2), wristWorld(2)], ...
      [elbowWorld(3), wristWorld(3)], 'r-', 'LineWidth', 4);
scatter3(base(1), base(2), base(3), 70, 'k', 'filled');
scatter3(shoulderWorld(1), shoulderWorld(2), shoulderWorld(3), ...
         70, 'b', 'filled');
scatter3(elbowWorld(1), elbowWorld(2), elbowWorld(3), ...
         70, [1.0, 0.5, 0.0], 'filled');
scatter3(wristWorld(1), wristWorld(2), wristWorld(3), ...
         80, 'r', 'filled');
scatter3(target(1), target(2), target(3), ...
         120, 'gx', 'LineWidth', 2.2);

text(base(1), base(2), base(3), '  base origin');
text(shoulderWorld(1), shoulderWorld(2), shoulderWorld(3), '  shoulder');
text(elbowWorld(1), elbowWorld(2), elbowWorld(3), '  elbow');
text(wristWorld(1), wristWorld(2), wristWorld(3), '  wrist/FK');
text(target(1), target(2), target(3), '  target');

quiver3(0, 0, 0, 80, 0, 0, 'Color', [0.1, 0.5, 0.1], ...
        'LineWidth', 1.5, 'MaxHeadSize', 0.6);
quiver3(0, 0, 0, 0, 80, 0, 'Color', [0.5, 0.1, 0.5], ...
        'LineWidth', 1.5, 'MaxHeadSize', 0.6);
quiver3(0, 0, 0, 0, 0, 80, 'Color', [0.1, 0.1, 0.5], ...
        'LineWidth', 1.5, 'MaxHeadSize', 0.6);
text(85, 0, 0, '+X');
text(0, 85, 0, '+Y');
text(0, 0, 85, '+Z');
view(45, 24);
xlim([-40, 230]);
ylim([-80, 100]);
zlim([-60, 190]);

% ---------------- X-Z side view in the arm plane ----------------
subplot(1, 2, 2);
hold on;
grid on;
axis equal;
xlabel('arm-plane X / radial direction (mm)');
ylabel('Z upward (mm)');
title(sprintf('Equal-scale X-Z view, q=[%.2f, %.2f, %.2f] deg', qDeg));

base2 = [0, 0];
shoulder2 = [shoulderLocal(1), shoulderLocal(3)];
elbow2 = [elbowLocal(1), elbowLocal(3)];
wrist2 = [wristLocal(1), wristLocal(3)];
target2 = [localX, target(3)];

plot([base2(1), shoulder2(1)], [base2(2), shoulder2(2)], ...
     'k--', 'LineWidth', 1.4);
plot([shoulder2(1), elbow2(1)], [shoulder2(2), elbow2(2)], ...
     'b-', 'LineWidth', 5);
plot([elbow2(1), wrist2(1)], [elbow2(2), wrist2(2)], ...
     'r-', 'LineWidth', 5);
plot([shoulder2(1), target2(1)], [shoulder2(2), target2(2)], ...
     ':', 'Color', [0.25, 0.25, 0.25], 'LineWidth', 1.2);

scatter(base2(1), base2(2), 70, 'k', 'filled');
scatter(shoulder2(1), shoulder2(2), 70, 'b', 'filled');
scatter(elbow2(1), elbow2(2), 70, [1.0, 0.5, 0.0], 'filled');
scatter(wrist2(1), wrist2(2), 80, 'r', 'filled');
scatter(target2(1), target2(2), 120, 'gx', 'LineWidth', 2.2);

text(base2(1), base2(2), '  base');
text(shoulder2(1), shoulder2(2), '  shoulder');
text(elbow2(1), elbow2(2), '  elbow');
text(wrist2(1), wrist2(2), '  wrist/FK');
text(target2(1), target2(2), '  target');

labelSegment(shoulder2, elbow2, sprintf('L1 = %.1f mm', L1), [0, 0, 1]);
labelSegment(elbow2, wrist2, sprintf('L2 = %.1f mm', L2), [1, 0, 0]);
labelSegment(base2, shoulder2, ...
             sprintf('shoulder Z = %.1f mm', shoulder(3)), [0, 0, 0]);
labelSegment(shoulder2, target2, ...
             sprintf('shoulder-target = %.1f mm', shoulderToTargetPlanar), ...
             [0.25, 0.25, 0.25]);

drawAngleArc(shoulder2, 38, 0, qDeg(2), ...
             sprintf('q2 = %.1f deg', qDeg(2)), [0, 0, 1]);
drawAngleArc(elbow2, 32, qDeg(2), qDeg(2) + qDeg(3), ...
             sprintf('signed q3 = %.1f deg', qDeg(3)), [1, 0, 0]);
drawAngleArc(elbow2, 48, 0, smallLinkPitchDeg, ...
             sprintf('q2+q3 = %.1f deg', smallLinkPitchDeg), ...
             [0.65, 0, 0.65]);
text(elbow2(1) + 20, elbow2(2) - 52, ...
     sprintf(['elbow opening/travel = %.1f deg\\n', ...
              'remaining to opposite = %.1f deg'], ...
             elbowOpenFromReferenceDeg, elbowToOppositeLimitDeg), ...
     'Color', [0.5, 0.05, 0.05], 'FontWeight', 'bold', ...
     'BackgroundColor', 'w', 'EdgeColor', [0.8, 0.5, 0.5], ...
     'Margin', 5);

plot([target2(1), target2(1)], [shoulder2(2), target2(2)], ...
     '--', 'Color', [0.6, 0.6, 0.6]);
plot([shoulder2(1), target2(1)], [target2(2), target2(2)], ...
     '--', 'Color', [0.6, 0.6, 0.6]);
text((shoulder2(1) + target2(1)) / 2, target2(2) - 8, ...
     sprintf('radial = %.1f mm', signedRadius), ...
     'HorizontalAlignment', 'center');
text(target2(1) + 4, (shoulder2(2) + target2(2)) / 2, ...
     sprintf('dz = %.1f mm', zPlanar));

annotationText = sprintf([ ...
    'vehicle target = [%.1f, %.1f, %.1f] mm\\n', ...
    'arm-plane target = [%.1f, %.1f] mm\\n', ...
    'horizontal rho = %.1f mm\\n', ...
    'q1 yaw compensation = %.2f deg\\n', ...
    'FK error = %.4f mm'], ...
    target(1), target(2), target(3), target2(1), target2(2), ...
    horizontalDistance, qDeg(1), fkError);
text(-30, 170, annotationText, ...
     'VerticalAlignment', 'top', 'BackgroundColor', 'w', ...
     'EdgeColor', [0.6, 0.6, 0.6], 'Margin', 6);

xlim([-45, 230]);
ylim([-70, 190]);

function labelSegment(p1, p2, label, color)
    mid = (p1 + p2) / 2;
    direction = p2 - p1;
    lengthValue = norm(direction);
    if lengthValue < eps
        offset = [0, 0];
    else
        normal = [-direction(2), direction(1)] / lengthValue;
        offset = normal * 8;
    end
    text(mid(1) + offset(1), mid(2) + offset(2), label, ...
         'Color', color, 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center', 'BackgroundColor', 'w');
end

function drawAngleArc(center, radius, startDeg, endDeg, label, color)
    if endDeg >= startDeg
        angleValues = linspace(startDeg, endDeg, 80);
    else
        angleValues = linspace(startDeg, endDeg, 80);
    end
    x = center(1) + radius * cosd(angleValues);
    z = center(2) + radius * sind(angleValues);
    plot(x, z, '-', 'Color', color, 'LineWidth', 1.8);

    midAngle = (startDeg + endDeg) / 2;
    labelPosition = center + ...
        [(radius + 14) * cosd(midAngle), (radius + 14) * sind(midAngle)];
    text(labelPosition(1), labelPosition(2), label, ...
         'Color', color, 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center', 'BackgroundColor', 'w');
end
