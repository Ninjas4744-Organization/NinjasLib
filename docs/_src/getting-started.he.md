# איך מתחילים

## התקנה

NinjasLib מופצת כקוד מקור, לא כחבילת Maven - מוסיפים אותה לפרויקט הרובוט כ-git submodule והיא
מתקמפלת יחד עם הקוד שלכם.

1. משורש פרויקט הרובוט, הוסיפו את ה-submodule תחת תיקיית `frc.lib`:

    ```bash
    git submodule add https://github.com/Ninjas4744-Organization/NinjasLib.git src/main/java/frc/lib/NinjasLib
    ```

2. NinjasLib תלויה בכמה vendor libraries של WPILib, ובשתי ספריות Java פשוטות. ודאו שבפרויקט שלכם
   מותקנים ה-vendordeps האלה (דרך הפקודה "Manage Vendor Libraries" ב-VS Code של WPILib, או על ידי
   הכנסת קבצי ה-JSON לתיקיית `vendordeps/`):

    - `WPILibNewCommands`
    - `Phoenix6` (CTRE TalonFX/Pigeon2)
    - `Phoenix5` (CTRE TalonSRX/VictorSPX) - רק אם משתמשים בבקרים האלה
    - `REVLib` (REV SparkMax) - רק אם משתמשים ב-`SparkMaxController`
    - `Studica` (NavX) - רק אם משתמשים בגירוסקופ NavX
    - `photonlib` (PhotonVision) - רק אם משתמשים בראייה ממוחשבת
    - `PathplannerLib` - רק אם משתמשים ב-`NinjasAutoBuilder` / אוטונומי לסוורב
    - `DogLog` - התיעוד של NinjasLib (`NinjasLogger`) בנוי ישירות מעליה
    - [`maple-sim`](https://github.com/Shenzhen-Robotics-Alliance/maple-sim) - סימולציית הפיזיקה
      שמאחורי המצב המדומה של `Swerve`

3. NinjasLib גם צריכה שתי תלויות Java פשוטות שאינן vendordeps - להוסיף אותן ישירות ל-`build.gradle`:

    ```groovy
    dependencies {
        implementation 'org.jgrapht:jgrapht-core:1.5.2' // בשימוש ב-StateMachineBase
        implementation 'org.dyn4j:dyn4j:5.0.2'          // בשימוש ב-maple-sim
    }
    ```

4. הריצו build. אם משהו חסר, שגיאת הקומפילציה תצביע על המחלקה הספציפית, מה שיגיד לכם איזה
   vendordep דילגתם עליו.

## הצורה של רובוט מבוסס NinjasLib

NinjasLib לא כופה מבנה פרויקט - אתם עדיין כותבים מחלקות `SubsystemBase` ו-`Command` רגילות. אבל
המודולים המרכזיים שלה (`Swerve`, `SwerveController`, `Vision`) הם **סינגלטונים** עם `periodic()`
משלהם שצריך לרוץ בכל לולאה, אז הדרך האידיומטית להשתמש בהם היא **להחזיק אחד מהם בתוך תת-מערכת
משלכם**, במקום לבנות אותם חופשיים ב-`RobotContainer`. הבנאי של תת-המערכת שלכם בונה ורושם את
הסינגלטון, וה-`periodic()` שלה מפעיל אותו - בדיוק כמו שהיא הייתה עושה עבור `Controller` או כל חלק
אחר של חומרה שהיא מחזיקה.

שתי תת-מערכות עולות כמעט בכל רובוט מבוסס NinjasLib: תת-מערכת הינע שעוטפת את
`Swerve`/`SwerveController`, ותת-מערכת ראייה שעוטפת את `Vision`. הראשונה היא באופן טבעי
[מכונת מצבים](guides/state-machine.md) - נהיגה פירושה להיות במדויק ב"מצב" אחד בכל רגע (בשליטת נהג,
עוקב אחר מסלול אוטונומי, מכוון למטרה, ...) - אז היא נבנית כ-`StateMachineBase` במקום
`SubsystemBase` פשוטה:

```java
public class SwerveSubsystem extends StateMachineBase<SwerveSubsystem.SwerveState> {
    public enum SwerveState { DRIVER, AUTO }

    private final DoubleSupplier driverLeftX, driverLeftY, driverRightX, driverRightY;
    private SwerveSpeeds autoInput = new SwerveSpeeds();

    public SwerveSubsystem(DoubleSupplier driverLeftX, DoubleSupplier driverLeftY,
                            DoubleSupplier driverRightX, DoubleSupplier driverRightY) {
        super(SwerveState.class);
        currentState = SwerveState.DRIVER;

        this.driverLeftX = driverLeftX;
        this.driverLeftY = driverLeftY;
        this.driverRightX = driverRightX;
        this.driverRightY = driverRightY;

        // תת-המערכת הזאת מחזיקה את Swerve ו-SwerveController: היא בונה אותם פעם אחת, כאן.
        Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
        SwerveController.setInstance(new SwerveController(SubsystemConstants.kSwerveController));
        SwerveController.get().setChannel("Driver");
    }

    @Override
    protected void define() {
        // המצב הפעיל, יהיה אשר יהיה, מזין את SwerveController ברצף דרך הערוץ שלו;
        // omni-edge מאפשר לכל מצב אחר לעבור אליו על פי דרישה.
        addOmniEdge(SwerveState.DRIVER, () -> Commands.runOnce(() -> SwerveController.get().setChannel("Driver")));
        addStateCommand(SwerveState.DRIVER, Commands.run(() -> SwerveController.get().setControl(getDriverInput(), "Driver")));

        addOmniEdge(SwerveState.AUTO, () -> Commands.runOnce(() -> SwerveController.get().setChannel("Auto")));
        addStateCommand(SwerveState.AUTO, Commands.run(() -> SwerveController.get().setControl(autoInput, "Auto")));
    }

    private SwerveSpeeds getDriverInput() {
        return new SwerveSpeeds(driverLeftY.getAsDouble(), driverLeftX.getAsDouble(),
            driverRightX.getAsDouble(), GeneralConstants.Swerve.kDriverFieldRelative);
    }

    /** נקרא על ידי PathPlanner/NinjasAutoBuilder בזמן שמסלול אוטונומי רץ. */
    public void setAutoInput(ChassisSpeeds speeds) {
        autoInput = new SwerveSpeeds(speeds, false);
    }

    @Override
    public void periodic() {
        SwerveController.get().periodic(); // מפעיל גם את Swerve.periodic(), שמזין את RobotPose
        super.periodic();
    }
}
```

זאת גרסה מכוונת-מקוצרת - ראו [מכונות מצבים](guides/state-machine.md) להבנת למה הדפוס הזה
(omni-edge לתוך מצב, בתוספת state command שמזין אותו ברציפות) הוא הצורה הנכונה ל"ההינע תמיד בדיוק
במצב אחד," ו[שילדת סוורב](guides/swerve.md) לכל מה ש-`Swerve`/`SwerveController` עצמם מציעים.
תת-מערכת הינע אמיתית בדרך כלל גדלה עם עוד מצבים באותו האופן (כיוון למטרה, נעילה לזווית קבועה,
מעקב אחר יעד ניקוד, ...), כולם חולקים את אותו שער ערוץ של `SwerveController` כך שבדיוק אחד מהם
נוהג בכל רגע.

`Vision` לא צריכה להיות מכונת מצבים - `SubsystemBase` פשוטה שמחזיקה אותה ומזינה את `RobotPose`
בכל לולאה מספיקה:

```java
public class VisionSubsystem extends SubsystemBase {
    public VisionSubsystem() {
        Vision.setInstance(new Vision(SubsystemConstants.kVision));
    }

    @Override
    public void periodic() {
        Vision.get().periodic();

        for (VisionOutput output : Vision.get().getVisionOutputs())
            RobotPose.get().addVisionUpdate(output, Timer.getFPGATimestamp());
    }
}
```

`RobotPose` עצמה לא מוחזקת על ידי אף אחת מתת-המערכות - אין לה `periodic()` משלה (`Swerve` מזינה
אותה אודומטריה ישירות מתוך ה-`periodic()` שלה), אז הכי פשוט לבנות אותה פעם אחת ב-`RobotContainer`,
יחד עם תת-המערכות שלכם:

```java
public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubsystem;

    public RobotContainer() {
        swerveSubsystem = new SwerveSubsystem(
            driverController::getLeftX, driverController::getLeftY,
            driverController::getRightX, driverController::getRightY);

        RobotPose.setInstance(new RobotPose(
            SubsystemConstants.kSwerve.chassis.kinematics,
            VisionStrengthCalculator.ninjasFunction(() -> odometryDrift),
            output -> output.ambiguity < 0.2
        ));

        visionSubsystem = new VisionSubsystem();

        // ... בונים את שאר תת-המערכות שלכם כרגיל
    }
}
```

כיוון ש-`SwerveSubsystem` ו-`VisionSubsystem` הן `SubsystemBase` (וגם `StateMachineBase` היא
כזאת), ה-`CommandScheduler` של WPILib כבר קורא ל-`periodic()` שלהן בכל לולאה אוטומטית ברגע שהן
נבנות - אין צריך לקרוא ל-`Swerve.get().periodic()`/`Vision.get().periodic()` בעצמכם מתוך
`Robot.robotPeriodic()`; זאת בדיוק הסיבה שה-`periodic()` של כל תת-מערכת למעלה קורא למודול שהיא
מחזיקה.

כל אחד מהסינגלטונים של `Swerve`/`SwerveController`/`Vision`/`RobotPose` בטוח לדלג עליו לגמרי: אם
אף פעם לא קוראים ל-`setInstance`, קריאה ל-`get()` עדיין מחזירה placeholder מנוטרל במקום לזרוק
חריגה, כך שרובוט בלי ראייה ממוחשבת, למשל, פשוט אף פעם לא בונה `VisionSubsystem`, וקריאה בטעות
ל-`Vision.get()` במקום אחר מתדרדרת בעדינות במקום לקרוס.

## לאן להמשיך

- לנהוג עם הרובוט: [שילדת סוורב](guides/swerve.md)
- להגדיר מנוע/ים של מנגנון: [בקרי מנועים](guides/controllers.md)
- למדל התנהגות של תת-מערכת כמצבים: [מכונות מצבים](guides/state-machine.md)
- לדעת איפה הרובוט נמצא על המגרש: [מיקום וראייה ממוחשבת](guides/localization.md)
