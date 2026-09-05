# בקרי מנועים

`Controller` היא עטיפה אחידה מעל מנגנון מנוע בודד (עם followers אופציונליים): בקרת פלט אחוזי, מיקום,
ומהירות, בתוספת קריאה חזרה של אנקודר/זרם/מתגי הגבלה. כותבים את תת-המערכת שלכם מול ה-API של
`Controller` פעם אחת, וקוד זהה מפעיל TalonFX, SparkMax, TalonSRX, VictorSPX אמיתיים, או סימולציית
תוכנה טהורה - תלוי רק באיזה `ControllerType` בוחרים ובאם `Robot.isReal()`.

## יצירת בקר

תמיד להשתמש בשיטת ה-factory במקום לבנות תת-מחלקה ישירות - היא זאת שעוברת בין חומרה אמיתית ל-
`SimulatedController` בשבילכם:

```java
Controller elevatorMotor = Controller.createController(
    Controller.ControllerType.TalonFX,
    SubsystemConstants.kElevatorMotor
);
```

## הגדרת `ControllerConstants` / `RealControllerConstants`

`ControllerConstants` עוטפת שני צדדים של הגדרה - חומרה אמיתית (`RealControllerConstants`) וסימולציה
- אבל בפועל מגדירים את הצד `real`, והבקר המדומה משתמש בחלקים הרלוונטיים ממנו:

```java
public static final ControllerConstants kElevatorMotor = new ControllerConstants();
static {
    kElevatorMotor.real
        .withBase(new RealControllerConstants.Base()
            .withMain(new RealControllerConstants.Base.SimpleControllerConstants(20, false))
            .withFollowers(new RealControllerConstants.Base.SimpleControllerConstants[] {
                new RealControllerConstants.Base.SimpleControllerConstants(21, true)
            })
            .withIsBrakeMode(true)
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(80))
        .withControl(new RealControllerConstants.Control()
            .withControlConstants(ControlConstants.createProfiledPIDF(
                4.0, 0, 0, 0,       // P, I, D, IZone
                80, 160, 0,         // cruise velocity, acceleration, jerk
                0, 0.4, 0.1, 0.3, GravityTypeValue.Elevator_Static))
            .withConversion(12.0, 1)          // יחס הילוכים, ואז המרת סיבובים ליחידות פלט
            .withPositionGoalTolerance(0.02))
        .withSoftLimits(new RealControllerConstants.SoftLimits().withMin(0).withMax(1.6))
        .withHardLimits(new RealControllerConstants.HardLimits.HardLimit[] {
            new RealControllerConstants.HardLimits.HardLimit()
                .withId(0)
                .withDirection(-1)
                .withHomePosition(0)
        });
}
```

שדות שכדאי להכיר:

- **`control.gearRatio` / `control.conversionFactor`** הופכים סיבובי מנוע גולמיים ליחידות מנגנון:
  `output = motorRotations / gearRatio * conversionFactor`. אם רוצים ש-`getPosition()` של מעלית
  יהיה במטרים, `conversionFactor` הוא היקף התוף; אם רוצים זרוע במעלות, `conversionFactor` הוא `360`.
- **`control.controlConstants`** נבנה עם אחת מהשיטות `ControlConstants.createPID(...)`,
  `createPIDF(...)`, `createProfile(...)`, או `createProfiledPIDF(...)`, בהתאם להתנהגות הבקרה
  הרצויה (PID רגיל מול פרופיל תנועה, עם או בלי feedforward).
- **`hardLimits.limits`** מתאר מתגי הגבלה - אמיתיים המחוברים ל-`DigitalInput`, או *וירטואליים*
  (`.withVirtual(true, stallCurrentThreshold)`) שמוסקים אך ורק מזרם המנוע, למנגנונים בלי מתג פיזי.
  לכל הגבלה יש `limitTriggerMethod` (ברירת מחדל: "אפס מחדש את האנקודר והחזק מיקום") שמופעל
  אוטומטית מ-`Controller.periodic()` כשההגבלה נדלקת מחדש.
- **`canCoder`** (דרך `.withId(...)` על `RealControllerConstants#canCoder`) מחבר CANcoder של CTRE
  למשוב מיקום אבסולוטי, באחד משלושה מצבים - `Normal` (ניתן לקריאה/איפוס מקוד), `Fused`, או `Sync`
  (ראו ב-Javadoc על `CANCoderMode` את הפשרות בין השלושה).

## הפעלת המנגנון

```java
public class ElevatorSubsystem extends SubsystemBase {
    private final Controller motor = Controller.createController(
        Controller.ControllerType.TalonFX, SubsystemConstants.kElevatorMotor);

    public void setPercent(double percent) { motor.setPercent(percent); }
    public void setHeight(double meters)    { motor.setPosition(meters); }
    public boolean atGoal()                 { return motor.atGoal(); }

    @Override
    public void periodic() {
        motor.periodic(); // מבצע דיבאונס למתגי הגבלה ומפעיל limitTriggerMethod
    }
}
```

`setPercent`/`setPosition`/`setVelocity` כל אחת מחליפה את ה-`ControlState` הפנימי של הבקר; תת-המחלקה
הקונקרטית (למשל `TalonFXController`) מפעילה בפועל את ה-PID/Motion Magic/feedforward לפי הגברים
מ-`RealControllerConstants.Control`. `atGoal()` משווה את המיקום או המהירות הנוכחיים מול הסבילות
המוגדרת, תלוי באיזה מצב בקרה פעיל כרגע - היא תמיד מחזירה `false` במצב פלט אחוזי.

## קריאת מצב בחזרה

```java
motor.getPosition();       // יחידות מנגנון, לפי conversionFactor
motor.getVelocity();       // יחידות מנגנון לשנייה
motor.getSupplyCurrent();  // אמפרים שנשאבים מהסוללה/באס
motor.getStatorCurrent();  // אמפרים דרך סלילי המנוע
motor.getLimit();          // true אם מתג הגבלה כלשהו פעיל
motor.getLimit(0);         // true אם מתג הגבלה באינדקס 0 ספציפית פעיל
```

לטלמטריה, `motor.getLogs()` לוקח תמונת מצב של כל אלה לתוך אובייקט `Controller.ControllerLogs` אחד
הניתן לסריאליזציה, מתאים לתיעוד פעם אחת בכל לולאה.

## Homing מול מתג הגבלה

ברירת המחדל של `limitTriggerMethod` כבר מאפסת מחדש את האנקודר ל-`homePosition` בפעם הראשונה
שהגבלה הופכת לפעילה, ומחזיקה את המנגנון שם כל עוד הוא עדיין מונע לתוך ההגבלה - מה שמכסה את רוב
שגרות ה-homing מסוג "סע למטה עד שמתג ההגבלה נלחץ, ואז זה אפס" בלי קוד נוסף. דורסים את
`withLimitTriggerMethod(...)` רק אם צריך התנהגות מותאמת אישית (למשל גם לשנות את המצב של תת-המערכת
עצמה) כשהגבלה מופעלת.
