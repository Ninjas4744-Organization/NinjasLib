# כלי עזר

חלקים קטנים יותר של NinjasLib שלא צריכים מדריך שלם משלהם, מקובצים כאן לפי מה שהם עושים.

## `NinjasLogger`

`NinjasLogger extends DogLog`, כך שכל שיטת תיעוד של [DogLog](https://github.com/DogLogger/DogLog)
(`NinjasLogger.log(key, value)`) עובדת בדיוק כמו ב-DogLog - זה מה שכל מודול ב-NinjasLib משתמש בו
פנימית, וזה מה שכדאי להשתמש בו גם בתת-המערכות שלכם לצורך עקביות. מעבר לזה, היא מוסיפה תיעוד
אירועים ממוספר:

```java
NinjasLogger.logEvent("Intake started");
// מדפיס "[3] Intake started" לקונסולה ומתעד אותו תחת "Event"
```

כל אירוע מקבל מזהה קצר וקומפקטי שעולה (בסיס 22, למשל `A`, `B`, ..., `K`, `10`, ...) כך שאפשר לדעת
באיזה סדר אירועים קרו גם אם חותמות הזמן של DogLog לא מדויקות מספיק כדי להבחין בין שני אירועים
באותה לולאה.

```java
NinjasLogger.logEventImportant("Vision instance not set.");
```

`logEventImportant` מיועד לבעיות שלא רוצים שנהג/מתכנת יפספסו במהלך מקצה: היא מדווחת את ההודעה
כשגיאת `DriverStation` (שמופיעה בבירור בקונסולת ה-Driver Station) ומתעדת אותה כמה פעמים ברצף
במקום פעם אחת. זה מה שכל ברירת מחדל של סינגלטון מנוטרל בספרייה (`Swerve.get()`, `RobotPose.get()`,
`Vision.get()`, ...) משתמש בו כדי להזהיר שאף פעם לא קראתם ל-`setInstance`.

## `NinjasAutoBuilder`

עטיפה דקה מעל ה-`AutoBuilder` של PathPlanner שחושפת רק בנייה/בחירה של אוטונומי, עם שני הבדלים
משימוש ישיר ב-`AutoBuilder`: `buildAuto` נותן לבחור אם למראה (mirror) את האוטונומי לצד השני של
המגרש, והבוחרים (choosers) מחזירים את **השם** של האוטונומי שנבחר במקום פקודה בנויה, כך שבונים אותו
בעצמכם ברגע שהאוטונומי בפועל מתחיל (עם המראה שרוצים באותו רגע):

```java
// פעם אחת, בזמן ההגדרה (אחרי שה-AutoBuilder.configure(...) של PathPlanner עצמו כבר נקרא):
SendableChooser<String> autoChooser = NinjasAutoBuilder.buildAutoChooser("MyDefaultAuto");
SmartDashboard.putData("Auto Chooser", autoChooser);

// ב-autonomousInit():
String chosenAuto = autoChooser.getSelected();
if (!chosenAuto.isEmpty()) {
    boolean shouldMirror = RobotPose.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    PathPlannerAuto auto = NinjasAutoBuilder.buildAuto(chosenAuto, shouldMirror);
    auto.schedule();
}
```

`buildAutoChooserWithOptionsModifier` גם נותן לשנות את רשימת שמות האוטונומי לפני שהם נוספים
לבוחר (למשל לסנן אוטונומי לבדיקות בלבד, או לעצב מחדש שמות תצוגה).

## `DerivativeCalculator` / `DerivativeCalculator2d`

עזר קטן ובעל מצב (stateful) להפוך ערך נדגם רועש לקצב שינוי מוחלק, למשל להעריך תאוצה מאות מהירות,
או מהירות מאות מיקום, בלי לכתוב מסנן low-pass משלכם בכל פעם:

```java
private final DerivativeCalculator accelCalculator = new DerivativeCalculator(8); // חלון ממוצע

@Override
public void periodic() {
    double acceleration = accelCalculator.calculate(getVelocity());
}
```

זה מסנן ממוצע נע מעל הנגזרת הגולמית בין דגימה לדגימה: `averageWindow` גדול יותר חלק יותר אבל עם
יותר עיכוב (lag) - 5–10 דגימות היא נקודת התחלה סבירה. כיוון שהוא שומר מצב בין קריאות (הערך/חותמת
הזמן האחרונים), לקרוא לו פעם אחת בכל לולאה עם אות עקבי, ולא לשתף מופע אחד בין ערכים לא קשורים.
`DerivativeCalculator2d` הוא אותו רעיון עבור אות `Translation2d`, למשל גזירת מיקום דו-ממדי לוקטור
מהירות.
