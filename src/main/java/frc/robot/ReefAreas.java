package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;

public enum ReefAreas {
  Red6(new double[][]{
    {17.55 - 4.319, 3.67}, 
    {17.55 - -0.191, 0.67}, 
    {17.55 - 4.319, -0.33}},
    Red,
    120),
  Red7(new double[][]{
    {17.55 - 4.12, 4.0}, 
    {17.55 - -0.38, 7.0}, 
    {17.55 - -0.38, 1.0}},
    Red,
    180),
  Red8(new double[][]{
    {17.55 - 4.319, 4.33},
    {17.55 - 4.319, 8.33},
    {17.55 - -0.191, 7.33}},
    Red,
    240),
  Red9(new double[][]{
    {17.55 - 4.691, 4.33},
    {17.55 - 10.191, 7.33},
    {17.55 - 4.691, 8.33}},
    Red,
    300),
  Red10(new double[][]{
    {17.55 - 4.88, 4.0},
    {17.55 - 10.38, 1.0},
    {17.55 - 10.38, 7.0}},
    Red,
    0),
  Red11(new double[][]{
    {17.55 - 4.691, 3.67},
    {17.55 - 4.691, -0.33},
    {17.55 - 10.191, 0.67}},
    Red,
    60),
  Blue17(new double[][]{
    {4.319, 3.67},
    {4.319, -0.33},
    {-0.191, 0.67}},
    Blue,
    60),
  Blue18(new double[][]{
    {4.12, 4.0},
    {-0.38, 1.0},
    {-0.38, 7.0}},
    Blue,
    0),
  Blue19(new double[][]{
    {4.319, 4.33},
    {-0.191, 7.33},
    {4.319, 8.33}},
    Blue,
    300),
  Blue20(new double[][]{
    {4.691, 4.33},
    {4.691, 8.33},
    {10.191, 7.33}},
    Blue,
    240),
  Blue21(new double[][]{
    {4.88, 4.0},
    {10.38, 7.0},
    {10.38, 1.0}},
    Blue,
    180),
  Blue22(new double[][]{
    {4.691, 3.67},
    {10.191, 0.67},
    {4.691, -0.33}},
    Blue,
    120);

  private final List<Pose2d> referencePoints;
  private final DriverStation.Alliance alliance;
  private final double angle;

  private ReefAreas(double[][] referencePoints, DriverStation.Alliance alliance, double angle) {
    this.referencePoints = Arrays.stream(referencePoints)
        .map(coord -> new Pose2d(coord[0], coord[1], new Rotation2d(0)))
        .collect(Collectors.toList());
    this.alliance = alliance;
    this.angle = angle;
  }

  public static Optional<Double> getAngle(Pose2d robotLocation, DriverStation.Alliance currentAlliance) {
    return Arrays.stream(ReefAreas.values())
        .filter(area -> area.alliance == currentAlliance)
        .filter(area -> isInArea(area.referencePoints, robotLocation))
        .findAny()
        .map(area -> area.angle);
  }

    /**s
   * Calculates if you are within a certain amount of points
   * @param border Array of points to calculate if you are inside of
   * @param estimatedPose2d Current robot position
   * @return Returns if you are in the given area
   */
  private static boolean isInArea(List<Pose2d> border, Pose2d estimatedPose2d) {
    double degree = 0.0;
    for (int i = 0 ; i < border.size(); i++) {
      Pose2d a = border.get(i);
      Pose2d b = border.get((i + 1) % (border.size()));
      double x1 = a.getX();
      double x2 = b.getX();
      double y1 = a.getY();
      double y2 = b.getY();
      double targetX = estimatedPose2d.getX();
      double targetY = estimatedPose2d.getY();

      // Calculate distance of vector
      double A = Math.sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
      double B = Math.sqrt((y1 - targetY) * (y1 - targetY) + (x1 - targetX) * (x1 - targetX));
      double C = Math.sqrt((y2 - targetY) * (y2 - targetY) + (x2 - targetX) * (x2 - targetX));

      // Calculate direction of vector
      double ta_x = x1 - targetX;
      double ta_y = y1 - targetY;
      double tb_x = x2 - targetX;
      double tb_y = y2 - targetY;

      double cross = tb_y * ta_x - tb_x * ta_y;
      boolean clockwise = cross < 0;

      // Calculate sum of angles
      if (clockwise) {
        degree = degree + Math.toDegrees(Math.acos((B * B + C * C - A * A) / (2.0 * B * C)));
      }
      else {
        degree = degree - Math.toDegrees(Math.acos((B * B + C * C - A * A) / (2.0 * B * C)));
      }
    }
    return Math.abs(Math.round(degree) - 360) <= 3;
  }
}
