# שילדת סוורב

מודול הסוורב הוא שתי מחלקות שפועלות יחד:

- **`Swerve`** - שילדת ההינע עצמה. מחזיקה את ארבעת המודולים והגירוסקופ, מפעילה הגבלת תאוצה ומהירות,
  ומזינה את האודומטריה. נבנית פעם אחת מאובייקט `SwerveConstants`.
- **`SwerveController`** - שכבה דקה מעל `Swerve` שמוסיפה PID לסיבוב/תזוזה ("תסתכל לכיוון הנקודה
  הזאת", "סע לנקודה הזאת"), ומנגנון שיתוף גישה (`setControl`/`setChannel`) כדי שכמה פקודות לא ילחמו
  זו בזו על השילדה בו-זמנית.

שתיהן סינגלטונים: בונים אותן פעם אחת, רושמים עם `setInstance`, וניגשים אליהן מכל מקום אחר דרך
`Swerve.get()` / `SwerveController.get()`.

## איפה כל חלק בקוד נמצא

כל קטע קוד בעמוד הזה שייך לאחד משני מקומות. הקבועים נמצאים בקובץ הקבועים של הרובוט, וכל מה שקורא
ל-`Swerve`/`SwerveController` נמצא בסאבסיסטם השילדה:

| קטע קוד | איפה הוא נמצא |
| --- | --- |
| `kDriveMotorConstants`, `kSteerMotorConstants`, `kSwerve`, `kSwerveController` | קובץ הקבועים שלכם, למשל `frc/robot/constants/SubsystemConstants.java`. מוגדרים כשדות `public static final`, בסדר הזה. |
| `Swerve.setInstance(...)`, `SwerveController.setInstance(...)` | ה**בנאי** של סאבסיסטם השילדה (למשל `SwerveSubsystem`), קודם `Swerve`. הסאבסיסטם עצמו נבנה פעם אחת ב-`RobotContainer`. |
| `Swerve.get().drive(...)`, `SwerveController.get().setControl(...)`, `lookAt`/`pidTo` | ה-`periodic()` של סאבסיסטם השילדה, או פקודה / פקודת מצב שהוא מחזיק - כל מקום שרץ בכל לולאה **אחרי** שהבנאי רץ. |
| `Swerve.get().periodic()` / `SwerveController.get().periodic()` | ה-`periodic()` של סאבסיסטם השילדה. אף אחד אחר לא קורא לזה בשבילכם. |
| קריאת מצב (`getGyro()`, `getSpeeds()`, ...) | כל מקום, אחרי שהאינסטנסים קיימים. |

[איך מתחילים](../getting-started.md) מראה סאבסיסטם שילדה שלם
עם כל החלקים האלה במקומם.

## הגדרת מנועי ה-drive וה-steer

לכל מודול יש מנוע drive אחד (מסובב את הגלגל) ומנוע steer אחד (מסובב את המודול). כל ארבעת מנועי
ה-drive חולקים אובייקט `ControllerConstants` אחד, וכל מנועי ה-steer חולקים אחר - כאן
`kDriveMotorConstants` ו-`kSteerMotorConstants`. אלה [קבועי בקר מנוע](controllers.md) רגילים, אז כל
מה שכתוב שם תקף גם כאן; החלק הזה מכסה את מה שספציפי לסוורב.

**לא מגדירים כאן מזהי מנועים או היפוכים.** הם שונים בכל מודול, ולכן באים מ-`SwerveModuleConstants`
(ראו בהמשך); הספרייה משכפלת את הקבועים המשותפים לכל מודול ומשלימה את המזהים וההיפוך של אותו
מודול. ה-CAN bus מגיע מ-`special.CANBus`, וגם ה-CANcoder של המודול מוגדר מקבועי המודול, אז אל
תיגעו ב-`canCoder`, ב-followers ובהגבלות רכות/קשות.

מה כן מגדירים:

- **`control.gearRatio` ו-`control.conversionFactor`** קובעים באילו יחידות הספרייה עובדת, והן חייבות
  להתאים למה ש-`Swerve` מצפה:
    - **Drive:** מטרים ומטרים לשנייה. `gearRatio` הוא יחס ההעברה של ה-drive במודול (למשל `6.75`
      ב-SDS MK4i L2) ו-`conversionFactor` הוא היקף הגלגל, `2 * Math.PI * wheelRadiusMeters`.
    - **Steer:** רדיאנים. `gearRatio` הוא יחס ההעברה של ה-steer במודול (למשל `150.0 / 7` ב-MK4i)
      ו-`conversionFactor` הוא `2 * Math.PI`.
- **`control.controlConstants`** הם הגברים. מנוע ה-drive פועל בבקרת *מהירות* בלולאה סגורה, לכן
  משתמשים ב-`ControlConstants.createPIDF(...)` עם feed-forward למהירות `V` בערך `12 / maxSpeed` (וולט
  למ'/ש') ו-`S` קטן לחיכוך סטטי; `A` ו-`G` הם `0`, וארגומנט `GravityTypeValue` חובה אבל אין לו השפעה
  על שילדת הינע. מנוע ה-steer פועל בבקרת *מיקום* בלולאה סגורה, אז מספיק
  `ControlConstants.createPID(P, I, D, IZone)` רגיל. הספרייה מטפלת בעצמה במעבר ±180°, אז לא מגדירים
  קלט רציף.
- **מגבלות הזרם ב-`base`** - מנוע ה-drive צריך מגבלת סטטור גבוהה יותר (הוא מספק את המומנט
  להאצת הרובוט) ממנוע ה-steer.
- **סוג הבקר** - `Controller.ControllerType.TalonFX` וכו' - *אינו* חלק מהקבועים; מעבירים אותו לצידם
  ב-`withDriveMotor(...)`/`withSteerMotor(...)`. מנועי ה-drive וה-steer יכולים להיות מסוגים שונים,
  אבל ת'רד האודומטריה בתדירות גבוהה (`special.enableOdometryThread`) דורש ששניהם יהיו TalonFX.

```java
public class SubsystemConstants {
    private static final double kWheelRadius = 0.049; // מטרים

    // להגדיר את אלה *מעל* kSwerve: שדות סטטיים מאותחלים מלמעלה למטה, ולכן שדה שמוגדר
    // למטה עדיין יהיה null כש-kSwerve קורא אותו.
    public static final ControllerConstants kDriveMotorConstants = new ControllerConstants();
    public static final ControllerConstants kSteerMotorConstants = new ControllerConstants();

    static {
        // Drive: בקרת מהירות, במטרים ומטרים לשנייה
        kDriveMotorConstants.real
            .withBase(new RealControllerConstants.Base()
                .withStatorCurrentLimit(100)
                .withSupplyCurrentLimit(60))
            .withControl(new RealControllerConstants.Control()
                .withConversion(6.75, 2 * Math.PI * kWheelRadius)     // יחס העברה, היקף
                .withControlConstants(ControlConstants.createPIDF(
                    1.0, 0, 0, 0,                                     // P, I, D, IZone
                    2.35, 0, 0.3, 0, GravityTypeValue.Elevator_Static))); // V, A, S, G, סוג כוח המשיכה

        // Steer: בקרת מיקום, ברדיאנים
        kSteerMotorConstants.real
            .withBase(new RealControllerConstants.Base()
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimit(30))
            .withControl(new RealControllerConstants.Control()
                .withConversion(150.0 / 7, 2 * Math.PI)               // יחס העברה, רדיאנים לסיבוב
                .withControlConstants(ControlConstants.createPID(25, 0, 0.25, 0)));
    }

    // public static final SwerveConstants kSwerve = ... (בחלק הבא)
}
```

בסימולציה נקראים מהאובייקטים האלה רק הגברים `P`, `I`, `D` ו-`IZone`. המודל הפיזי מגיע מ-
`SwerveConstants.simulation`, ולכן יחס ההעברה ופקטור ההמרה למעלה מתעלמים בסימולציה - מגדירים אותם
בשביל הרובוט האמיתי, ומכוונים את הגברים בנפרד אם הסימולציה נוסעת אחרת.

## הגדרת `SwerveConstants`

`SwerveConstants` מקבצת הגדרות לתוך אובייקטים מקוננים, כל אחד עם setterים שרשורים בשם `withX(...)`.
צריך לדרוס רק את מה שונה מברירת המחדל. זה נכתב באותו קובץ קבועים, מתחת לקבועי המנועים שהוא מפנה
אליהם:

```java
public class SubsystemConstants {
    // kDriveMotorConstants ו-kSteerMotorConstants מלמעלה

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
  steerInverted, canCoderID, canCoderInverted, canCoderOffset)`. המזהים וההיפוכים כאן הם אלה שבהם
  מנועי כל מודול משתמשים בפועל (הם דורסים את הערכים הזמניים בקבועי המנועים המשותפים), וההיסט הוא
  magnet offset של ה-CANcoder ברוטציות, מכויל כך שהמודול קורא אפס כשהגלגל מצביע קדימה. סדר
  המודולים (אינדקסים 0–3) חייב להתאים לסדר הפינות שבו משתמש `chassis.kinematics`: קדמי שמאל, קדמי
  ימין, אחורי שמאל, אחורי ימין.
- **`withSimulation(...)`** (לא מופיע למעלה) מגדיר את סוג המודול המדומה, למשל
  `new SwerveConstants.Simulation().withSwerveType(SwerveConstants.Simulation.SwerveType.Mark4i, 2)`;
  משמש רק בסימולציה, וברירות המחדל טובות עד שרוצים שהסימולציה תתאים למודולים האמיתיים שלכם.
- **`special.enableOdometryThread`** (דרך `.withOdometryThread(frequency)`) מריץ דגימת אודומטריה
  בת'רד ייעודי בתדירות גבוהה במקום בלולאה הראשית של 50 הרץ - שימושי אם צריך עדכוני מיקום מדויקים
  יותר לחישוב כמו ירי תוך כדי תנועה. השאירו כבוי אלא אם יש סיבה ספציפית להפעיל.
- **`special.enableAutoLock`** (דרך `.withAutoLock(frames)`) גורם ל-`drive()` לנעול אוטומטית את
  הגלגלים בצורת X אחרי שהנהג מחזיק קלט אפס למשך כמות הפריימים הזאת, כדי לעמוד מול דחיפה. כבוי
  כברירת מחדל.

## בנייה ורישום של `Swerve`

עושים את זה פעם אחת, בבנאי של סאבסיסטם השילדה, לפני שמשהו קורא ל-`Swerve.get()`:

```java
public class SwerveSubsystem extends SubsystemBase {
    public SwerveSubsystem() {
        Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
        // SwerveController.setInstance(...) מגיע מיד אחרי, ראו בהמשך
    }
}
```

הבנאי בודק את `Robot.isReal()` ובונה או מודולי `SwerveModuleIOReal` אמיתיים עם `GyroIONavX`/
`GyroIOPigeon2`, או `SwerveDriveSimulation` של MapleSim עם מודולים וגירוסקופ מדומים - הקוד שלכם
אף פעם לא צריך להבחין בעצמו בין אמיתי לסימולציה.

## נהיגה

בכל לולאה, מזינים את התנועה הרצויה ל-`drive()` בתור `SwerveSpeeds` - תת-מחלקה של `ChassisSpeeds`
של הספרייה, שנושאת גם דגל `fieldRelative`. זה נכתב ב-`periodic()` של סאבסיסטם השילדה (זו הצורה
הפשוטה ביותר; גרסת ה-state machine ב-[איך מתחילים](../getting-started.md) קוראת במקום זה
ל-`SwerveController.get().setControl(...)` מתוך פקודת מצב):

```java
public class SwerveSubsystem extends SubsystemBase {
    // ...הבנאי מלמעלה...

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

נקודות כניסה שימושיות נוספות, שאפשר לקרוא להן מכל פקודה או שיטה של הסאבסיסטם:

```java
Swerve.get().stop();              // בקשת נהיגה ריקה
Swerve.get().lockWheelsToX();     // נעילת גלגלים בצורת X, למשל כדי לעמוד מול דחיפה
Swerve.get().getSpeeds();         // מהירות נוכחית לפי אודומטריה
Swerve.get().getModuleStates();   // מצב כל מודול, לאבחון/תיעוד
```

## כיוון ונהיגה לנקודה עם `SwerveController`

`SwerveController` עוטף PID לסיבוב (רגיל או עם פרופיל תנועה, תלוי אם מגדירים cruise
velocity/acceleration) ו-PID לתזוזה, כדי שפקודות לא יצטרכו לממש כל אחת בעצמה "תסתובב מול X" או
"סע לכיוון X". הקבועים שלו נכתבים בקובץ הקבועים, *מתחת* ל-`kSwerve`, והוא נרשם בבנאי של סאבסיסטם
השילדה מיד אחרי `Swerve`:

```java
// SubsystemConstants.java
public static final SwerveControllerConstants kSwerveController = new SwerveControllerConstants()
    .withSwerveConstants(kSwerve)
    .withRotationPID(ControlConstants.createPID(4.0, 0, 0.1, 0))
    .withDrivePID(ControlConstants.createPID(3.0, 0, 0, 0));

// SwerveSubsystem.java, בבנאי
Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
SwerveController.setInstance(new SwerveController(SubsystemConstants.kSwerveController));
```

`rotationPIDConstants` הופך אוטומטית ל-PID עם פרופיל תנועה אם גם מגדירים
`cruiseVelocity`/`acceleration` (למשל דרך `ControlConstants.createProfiledPIDF(...)`) - אחרת זה
`PIDController` רגיל עם קלט רציף (continuous input).

ואז משתמשים בו מה-`periodic()` של סאבסיסטם השילדה, או מפקודה/פקודת מצב שרצה בכל לולאה:

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
מתפקד כמנגנון בעלות פשוט במקום להסתמך רק על מערכת ה-requirements של WPILib. קוראים לאלה מהפקודות שרוצות לנהוג - `setChannel` פעם אחת כשפקודה מתחילה (למשל ב-
`initialize()`, או ב-omni-edge אל מצב), `setControl` בכל לולאה כל עוד היא רצה:

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

מכל מקום בקוד, אחרי שסאבסיסטם השילדה נבנה:

```java
Swerve.get().getGyro().getYaw();      // כיוון גירוסקופ נוכחי
Swerve.get().getOdometryTwist();      // תזוזה מאז הקריאה האחרונה לשיטה הזאת
RobotPose.get().getRobotPose();       // מיקום מלא במגרש (ראו מיקום וראייה ממוחשבת)
```

ראו [מיקום וראייה ממוחשבת](localization.md) כדי להבין איך פלט האודומטריה של `Swerve` הופך למיקום
במגרש, ואיך ראייה ממוחשבת מתקנת אותו.
