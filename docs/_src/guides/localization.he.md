# מיקום וראייה ממוחשבת

שתי מחלקות פועלות יחד כדי לענות "איפה הרובוט על המגרש": **`RobotPose`** ממזגת אודומטריית סוורב
עם תיקוני ראייה ממוחשבת להערכת מיקום אחת, ו-**`Vision`** מנהלת כל מספר של מצלמות (Limelight או
PhotonVision, אמיתיות או מדומות) ומזינה את הערכות המיקום שלהן לתוכה. שתיהן סינגלטונים, נרשמות פעם
אחת עם `setInstance` כמו שאר הספרייה.

## `RobotPose`

`RobotPose` מחזיקה שני מעריכי מיקום בסנכרון:

- **`getRobotPose()`** - ההערכה הממוזגת עם ראייה ממוחשבת. להשתמש בזה בכל מקום כברירת מחדל (כיוון,
  תיעוד, אוטונומי).
- **`getOdometryOnlyRobotPose()`** - אודומטריה בלבד, מתעלמת מראייה ממוחשבת לחלוטין. להשתמש בזה רק
  כשאי-רציפות מתיקון ראייה תזיק, למשל הזנת לולאת בקרה מבוססת מהירות/תאוצה שמניחה אות מיקום רציף.
  היא סוטה עם הזמן, כיוון שהיא אף פעם לא מתקנת מול ראייה.

```java
RobotPose.setInstance(new RobotPose(
    SubsystemConstants.kSwerve.chassis.kinematics,
    VisionStrengthCalculator.ninjasFunction(() -> myOdometryDriftEstimate),
    output -> output.ambiguity < 0.2 && output.closestTargetDist < 4.0
));
```

לרוב לא צריך לקרוא לשיטות העדכון של `RobotPose` ישירות - `Swerve.periodic()` כבר קורא ל-
`addOdometryUpdate`/`addTimedOdometryUpdate` בשבילכם בכל לולאה. מה שכן מחברים בעצמכם הם שני
המחשבונים שמועברים לבנאי, שניהם `@FunctionalInterface` כך שלמדה מספיקה:

- **`VisionFiltersCalculator`** - `isPassed(VisionOutput)` מחליט האם מדידת ראייה נכנסת צריכה
  להידחות לגמרי (למשל רחוקה מדי, מעורפלת מדי, או לא סבירה פיזית בהתחשב במיקום הנוכחי) לפני שהיא
  בכלל מגיעה למעריך המיקום.
- **`VisionStrengthCalculator`** - `calculate(VisionOutput)` מחזיר מטריצת 3x1 של אמון לכל ציר
  (x, y, theta) שמבטא כמה לסמוך על מדידה ש*כן* עברה את הסינון. NinjasLib מגיעה עם שניים מוכנים:
  `VisionStrengthCalculator.kDefault` (אמון קבוע לכל מדידה) ו-`VisionStrengthCalculator.ninjasFunction(odometryDrift)`,
  ששוקל אמון לפי מרחק לתג הקרוב ביותר ולפי כמה האודומטריה שלכם כבר סטתה - כותבים למדה משלכם רק
  אם אף אחד מהם לא מתאים.

שאילתות נפוצות:

```java
RobotPose.get().getDistance(targetPose);      // מטרים, רובוט ליעד
RobotPose.get().getTransform(targetPose);     // dx, dy, dtheta, יחסי למגרש
RobotPose.get().getRotation();                // כיוון נוכחי יחסי למגרש
RobotPose.get().setRobotPose(knownPose);      // למשל בתחילת אוטונומי, ממיקום התחלה ידוע
RobotPose.get().resetGyro(Rotation2d.kZero);  // איפוס כיוון במקום, תוך שמירה על התזוזה הנוכחית
```

## `Vision`

`Vision` מחזיקה כל מצלמה שמתוארת באובייקט `VisionConstants` ובודקת את כולן פעם אחת בכל לולאה:

```java
public static final VisionConstants kVision = new VisionConstants()
    .withFieldLayoutGetter(ignoredTags -> Optional.of(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)))
    .withRobotPoseSupplier(() -> RobotPose.get().getRobotPose())
    .withPhotonVision("FrontCam", new Transform3d(/* היסט רובוט-למצלמה */))
    .withLimelight("limelight-back");

Vision.setInstance(new Vision(kVision));
```

אם `cameras`, `fieldLayoutGetter`, או `robotPoseSupplier` חסרים, `Vision` מנטרלת את עצמה לגמרי
במקום להיבנות חלקית - כל accessor אז מחזיר ברירת מחדל בלתי מזיקה ו-`periodic()` הוא no-op, כך
שרובוט שעדיין לא חיווט ראייה ממוחשבת לא צריך טיפול מיוחד במקום אחר. בסימולציה, `Vision` גם בונה
`VisionSystemSim` שנזרע מ-`fieldLayoutGetter` ומקדם אותו מ-`robotPoseSupplier` בכל לולאה, כך
שמצלמות מדומות רואות AprilTags בהתאמה למיקום המדומה של הרובוט.

```java
@Override
public void robotPeriodic() {
    Vision.get().periodic(); // המקום היחיד שבו מצלמות נבדקות בפועל
}
```

### הזנת ראייה ממוחשבת לתוך `RobotPose`

הדפוס הרגיל הוא: לקרוא את הפלט של כל מצלמה מ-`Vision`, ואז להעביר אותו ל-`RobotPose`:

```java
for (VisionOutput output : Vision.get().getVisionOutputs()) {
    RobotPose.get().addVisionUpdate(output, Timer.getFPGATimestamp());
}
```

`addVisionUpdate` לא עושה כלום אם הפלט לא מדווח על מטרות; אחרת הוא מפעיל את ה-`VisionFiltersCalculator`
שלכם, ואם המדידה עוברת, מחיל אותה דרך האמון שחישב ה-`VisionStrengthCalculator` שלכם. גם תוצאת
העבור/לא-עבור וגם האמון המחושב מתועדים לפי מצלמה בכל מקרה, כך שמדידה שנדחתה או שנוכה בחוזקה עדיין
נראית ביומן.

אם כבר החלטתם שמיקום מסוים אמין דרך אמצעי אחר, `addManualVisionUpdate` מחיל אותו ישירות, תוך
דילוג על שני המחשבונים.

### שאילתות נוספות לפי מצלמה

```java
Vision.get().hasTargets("FrontCam");
Vision.get().getClosestTargetDistance("FrontCam");
Vision.get().getCameraToClosestTargetTransform("FrontCam");
Vision.get().ignoreTag(4);    // להפסיק לסמוך על AprilTag ספציפי (למשל תג שידוע כפגום/הוזז)
Vision.get().unIgnoreTag(4);
```

## הכול ביחד

```java
public class RobotContainer {
    public RobotContainer() {
        Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
        RobotPose.setInstance(new RobotPose(
            SubsystemConstants.kSwerve.chassis.kinematics,
            new MyVisionStrengthCalculator(),
            new MyVisionFiltersCalculator()));
        Vision.setInstance(new Vision(SubsystemConstants.kVision));
    }
}

// Robot.java
@Override
public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    Swerve.get().periodic();  // מזין אודומטריה לתוך RobotPose
    Vision.get().periodic();  // בודק מצלמות

    for (VisionOutput output : Vision.get().getVisionOutputs())
        RobotPose.get().addVisionUpdate(output, Timer.getFPGATimestamp());
}
```
