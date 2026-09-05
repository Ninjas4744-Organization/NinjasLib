# שילדת סוורב

מודול הסוורב הוא שתי מחלקות שפועלות יחד:

- **`Swerve`** - שילדת ההינע עצמה. מחזיקה את ארבעת המודולים והגירוסקופ, מפעילה הגבלת תאוצה ומהירות,
  ומזינה את האודומטריה. נבנית פעם אחת מאובייקט `SwerveConstants`.
- **`SwerveController`** - שכבה דקה מעל `Swerve` שמוסיפה PID לסיבוב/תזוזה ("תסתכל לכיוון הנקודה
  הזאת", "סע לנקודה הזאת"), ומנגנון שיתוף גישה (`setControl`/`setChannel`) כדי שכמה פקודות לא ילחמו
  זו בזו על השילדה בו-זמנית.

שתיהן סינגלטונים: בונים אותן פעם אחת, רושמים עם `setInstance`, וניגשים אליהן מכל מקום אחר דרך
`Swerve.get()` / `SwerveController.get()`.

## הגדרת `SwerveConstants`

`SwerveConstants` מקבצת הגדרות לתוך אובייקטים מקוננים, כל אחד עם setterים שרשורים בשם `withX(...)`.
צריך לדרוס רק את מה שונה מברירת המחדל:

```java
public class SubsystemConstants {
    public static final SwerveConstants kSwerve = new SwerveConstants()
        .withChassis(new SwerveConstants.Chassis()
            .withDimensions(0.6, 0.6)      // מרווח בין המודולים, מרחק בין הגלגלים (מטרים)
            .withBumper(0.9, 0.9))         // רוחב הפגוש, אורך הפגוש (מטרים)
        .withSpeeds(new SwerveConstants.Speeds()
            .withMaxSpeeds(5.0, 9.0)                       // מקסימום פיזי: מ'/ש', רד'/ש'
            .withSpeedLimits(4.0, 7.0)                     // הגבלות נהיגה רכות: מ'/ש', רד'/ש'
            .withAccelerationLimits(20, 12, 15))           // תאוצת סקיד, קדימה, סיבוב
        .withModules(new SwerveConstants.Modules()
            .withModuleConstants(new SwerveModuleConstants[] {
                new SwerveModuleConstants(0, 1, 2, false, true, 3, false, 0.421),  // קדמי שמאל
                new SwerveModuleConstants(1, 4, 5, false, true, 6, false, 0.128),  // קדמי ימין
                new SwerveModuleConstants(2, 7, 8, false, true, 9, false, 0.355),  // אחורי שמאל
                new SwerveModuleConstants(3, 10, 11, false, true, 12, false, 0.009) // אחורי ימין
            })
            .withDriveMotor(kDriveMotorConstants, Controller.ControllerType.TalonFX)
            .withSteerMotor(kSteerMotorConstants, Controller.ControllerType.TalonFX))
        .withGyro(new SwerveConstants.Gyro(5, false, SwerveConstants.Gyro.GyroType.Pigeon2))
        .withSpecial(new SwerveConstants.Special()
            .withRobotConfig(RobotConfig.fromGUISettings())
            .withRobotStartPose(new Pose2d(3, 3, Rotation2d.kZero)));
}
```

כמה שדות ששווה להתעכב עליהם:

- **`speeds.maxSpeed` / `maxAngularVelocity`** הם היכולת הפיזית האמיתית של השילדה - משמשים לצמצום
  (desaturate) מהירויות המודולים. **`speedLimit` / `rotationSpeedLimit`** הן הגבלות ה*נהיגה* בפועל
  ש-`drive()` אוכף, שיכולות להיות נמוכות יותר (למשל "מצב איטי"). לשנות אותן בזמן ריצה עם
  `Swerve.get().setSpeedLimit(...)` / `setRotationSpeedLimit(...)`.
- **`maxSkidAcceleration` / `maxForwardAcceleration` / `rotationAccelerationLimit`** ברירת המחדל שלהם
  היא אינסוף (בלי הגבלה). מגדירים אותם אחרי שאיפיינתם את הרובוט, או משאירים כברירת מחדל אם לא צריך
  הגבלת תאוצה.
- **`SwerveModuleConstants`** מקבל `(moduleNumber, driveMotorID, steerMotorID, driveInverted,
  steerInverted, canCoderID, canCoderInverted, canCoderOffset)`. סדר המודולים (אינדקסים 0–3) חייב
  להתאים לסדר הפינות שבו משתמש `chassis.kinematics`.
- **`special.enableOdometryThread`** (דרך `.withOdometryThread(frequency)`) מריץ דגימת אודומטריה
  בת'רד ייעודי בתדירות גבוהה במקום בלולאה הראשית של 50 הרץ - שימושי אם צריך עדכוני מיקום מדויקים
  יותר לחישוב כמו ירי תוך כדי תנועה. השאירו כבוי אלא אם יש סיבה ספציפית להפעיל.
- **`special.enableAutoLock`** (דרך `.withAutoLock(frames)`) גורם ל-`drive()` לנעול אוטומטית את
  הגלגלים בצורת X אחרי שהנהג מחזיק קלט אפס למשך כמות הפריימים הזאת, כדי לעמוד מול דחיפה. כבוי
  כברירת מחדל.

## בנייה ורישום של `Swerve`

```java
Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
```

הבנאי בודק את `Robot.isReal()` ובונה או מודולי `SwerveModuleIOReal` אמיתיים עם `GyroIONavX`/
`GyroIOPigeon2`, או `SwerveDriveSimulation` של MapleSim עם מודולים וגירוסקופ מדומים - הקוד שלכם
אף פעם לא צריך להבחין בעצמו בין אמיתי לסימולציה.

## נהיגה

בכל לולאה, מזינים את התנועה הרצויה ל-`drive()` בתור `SwerveSpeeds` - תת-מחלקה של `ChassisSpeeds`
של הספרייה, שנושאת גם דגל `fieldRelative`:

```java
public class SwerveSubsystem extends SubsystemBase {
    @Override
    public void periodic() {
        SwerveSpeeds driverInput = new SwerveSpeeds(
            forwardMetersPerSecond,
            strafeMetersPerSecond,
            rotationRadiansPerSecond,
            /* fieldRelative = */ true
        );

        Swerve.get().drive(driverInput);
        Swerve.get().periodic(); // חייבים לקרוא בכל לולאה
    }
}
```

`drive()` לא מפעיל את המהירויות המבוקשות ישירות - הוא מתייחס אליהן כ*יעד* שמצב פנימי מאיץ אליו,
כשהוא כפוף להגבלות התאוצה והמהירות מ-`SwerveConstants`. זה מה שגורם לשילדה להרגיש חלקה גם עם קלט
ג'ויסטיק עצבני. ברובוט אמיתי, מהירויות השילדה הסופיות גם עוברות דיסקרטיזציה (`ChassisSpeeds.discretize`)
כדי לתקן את לולאת הפקודות בת 20 מ"ש.

נקודות כניסה שימושיות נוספות:

```java
Swerve.get().stop();              // בקשת נהיגה ריקה
Swerve.get().lockWheelsToX();     // נעילת גלגלים בצורת X, למשל כדי לעמוד מול דחיפה
Swerve.get().getSpeeds();         // מהירות נוכחית לפי אודומטריה
Swerve.get().getModuleStates();   // מצב כל מודול, לאבחון/תיעוד
```

## כיוון ונהיגה לנקודה עם `SwerveController`

`SwerveController` עוטף PID לסיבוב (רגיל או עם פרופיל תנועה, תלוי אם מגדירים cruise
velocity/acceleration) ו-PID לתזוזה, כדי שפקודות לא יצטרכו לממש כל אחת בעצמה "תסתובב מול X" או
"סע לכיוון X":

```java
public static final SwerveControllerConstants kSwerveController = new SwerveControllerConstants()
    .withSwerveConstants(kSwerve)
    .withRotationPID(ControlConstants.createPID(4.0, 0, 0.1, 0))
    .withDrivePID(ControlConstants.createPID(3.0, 0, 0, 0));

SwerveController.setInstance(new SwerveController(kSwerveController));
```

`rotationPIDConstants` הופך אוטומטית ל-PID עם פרופיל תנועה אם גם מגדירים
`cruiseVelocity`/`acceleration` (למשל דרך `ControlConstants.createProfiledPIDF(...)`) - אחרת זה
`PIDController` רגיל עם קלט רציף (continuous input).

```java
// להסתכל לזווית קבועה יחסית למגרש:
double omega = SwerveController.get().lookAt(Rotation2d.fromDegrees(90));

// להסתכל למיקום יעד יחסי למגרש (למשל, הספיקר), עם היסט אם המנגנון שצריך לכוון
// הוא לא החזית של הרובוט:
double omegaAtTarget = SwerveController.get().lookAt(targetPose, Rotation2d.kZero);

// לנסוע ישר לכיוון נקודה:
Translation2d velocityToTarget = SwerveController.get().pidTo(targetTranslation);

Swerve.get().drive(new SwerveSpeeds(velocityToTarget, omegaAtTarget, true));
```

`lookAt`/`pidTo` רק *מחשבים* מהירות - הם לא קוראים בעצמם ל-`Swerve.get().drive()`, כך שאפשר לשלב
פלט של PID סיבוב עם תזוזה שנשלטת על ידי הנהג, או להפך, באותו `SwerveSpeeds`.

### שיתוף בטוח של השילדה בין פקודות

כשכמה פקודות עלולות לרצות לנהוג בסוורב (שליטת נהג, כיוון אוטומטי, רצף ניקוד), `setControl`/`setChannel`
מתפקד כמנגנון בעלות פשוט במקום להסתמך רק על מערכת ה-requirements של WPILib:

```java
// מי שאמור כרגע להיות רשאי לנהוג תופס את הערוץ:
SwerveController.get().setChannel("AutoAim");

// כל נתיב קוד יכול לנסות לנהוג - רק הקריאה של בעל הערוץ הנוכחי משפיעה בפועל:
SwerveController.get().setControl(speeds, "AutoAim"); // מתבצע
SwerveController.get().setControl(speeds, "Teleop");  // מתעלם בשקט
```

זה שימושי במיוחד כשפקודה שבבעלות state machine/רקע צריכה *לפעמים* להשתלט על הנהיגה בלי לבטל
את הפקודת ברירת המחדל של הנהג.

## קריאת מצב בחזרה

```java
Swerve.get().getGyro().getYaw();      // כיוון גירוסקופ נוכחי
Swerve.get().getOdometryTwist();      // תזוזה מאז הקריאה האחרונה לשיטה הזאת
RobotPose.get().getRobotPose();       // מיקום מלא במגרש (ראו מיקום וראייה ממוחשבת)
```

ראו [מיקום וראייה ממוחשבת](localization.md) כדי להבין איך פלט האודומטריה של `Swerve` הופך למיקום
במגרש, ואיך ראייה ממוחשבת מתקנת אותו.
