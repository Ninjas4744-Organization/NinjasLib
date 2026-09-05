# קלט משלט (LoggedCommandController)

`LoggedCommandController` עוטף שלט PS5 או Xbox ונותן שני דברים ש-`CommandPS5Controller`/
`CommandXboxController` של WPILib לא נותנים: כל כפתור, כיוון POV, וציר מתועדים אוטומטית דרך
`NinjasLogger` בכל לולאה, וההבדל בין PS5 ל-Xbox מוסתר מאחורי API משותף אחד (`cross()`/`circle()`/...
ממופים אוטומטית ל-`A()`/`B()`/... תלוי לאיזה IO נתתם לו).

## הגדרה

```java
LoggedCommandController driver = new LoggedCommandController(
    "DriverController",
    new LoggedCommandControllerIOPS5(0)  // או LoggedCommandControllerIOXbox(port) לשלט Xbox
);
```

הפרמטר `name` הוא הקידומת של מפתח היומן ב-DogLog (`"DriverController/cross"`,
`"DriverController/leftX"`, ...); לקרוא ל-`periodic()` פעם אחת בכל לולאה, בדרך כלל יחד עם בדיקת
שלטים אחרים שלכם ב-`robotPeriodic()`:

```java
@Override
public void robotPeriodic() {
    driver.periodic();
}
```

## קישור כפתורים

כל כפתור/כיוון POV חשוף כ-`Trigger` רגיל של WPILib, כך שהוא מתקשר בדיוק כמו `CommandXboxController`
רגיל:

```java
driver.cross().onTrue(indexer.changeStateCommand(Indexer.State.INTAKING));
driver.R1().whileTrue(shooterCommand);
driver.povUp().onTrue(Commands.runOnce(() -> RobotPose.get().resetGyro(Rotation2d.kZero)));
```

כפתורי הפנים מכונים בשתי הצורות (`cross()`/`circle()`/`square()`/`triangle()` וגם
`A()`/`B()`/`X()`/`Y()`) כך שאפשר להשתמש בשם שמתאים לשלט הפיזי שיש לצוות הנהיגה שלכם בפועל, בלי
קשר לאיזה `IO` בניתם איתו.

## קריאת צירים

צירים הם getterים רגילים של `double` במקום `Trigger`, בטווח `[-1, 1]` לג'ויסטיקים ו-`[0, 1]`
לטריגרים האנלוגיים:

```java
double forward = -driver.getLeftY();
double strafe  = -driver.getLeftX();
double rotate  = -driver.getRightX();
double intakePower = driver.getR2Axis();
```

## איזה כפתור ממפה למה

| קטגוריה | שיטות |
|---|---|
| כפתורי פנים | `cross()`/`A()`, `circle()`/`B()`, `square()`/`X()`, `triangle()`/`Y()` |
| D-Pad | `povUp()`, `povDown()`, `povLeft()`, `povRight()` |
| במפרים / לחיצות ג'ויסטיק | `L1()`, `R1()`, `L3()`, `R3()` |
| טריגרים ככפתורים | `L2()`, `R2()` (סף דיגיטלי) |
| כפתורי מערכת | `create()`/back, `options()`/start, `ps()` (PS5 בלבד), `touchpad()` (PS5 בלבד) |
| צירים | `getLeftX()`, `getLeftY()`, `getRightX()`, `getRightY()`, `getL2Axis()`, `getR2Axis()` |

`ps()` ו-`touchpad()` תמיד לא פעילים בשלט Xbox (אין כפתור חומרה לקרוא), אז בטוח לקשר אותם ללא תנאי
אפילו בקוד משותף בין הגדרת PS5 ל-Xbox.
