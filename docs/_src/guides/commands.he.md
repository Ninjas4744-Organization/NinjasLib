# פקודות (Commands)

NinjasLib מגיעה עם כמה מחלקות `Command`/עזר קטנות שחוזרות מספיק כדי להצדיק שיתוף, במקום שכל
תת-מערכת תתלה במסגרת פקודות גדולה משלה.

## `StateEndCommand`

מתנהג כמו `SequentialCommandGroup` של WPILib, עם הבדל אחד: ברגע שהיא מסתיימת, `isFinished()`
ממשיך להחזיר `true` לתמיד במקום רק עד שהקבוצה מורצת מחדש. זה מה ש-
[`StateMachineBase.addStateEnd`](state-machine.md) מצפה לו, כיוון שהוא בודק `isFinished()` פעם
אחת בכל לולאה וצריך תשובה יציבה במקום כזו שעלולה לחזור ל-`false`.

```java
addStateEnd(State.INTAKING,
    new StateEndCommand(Commands.waitUntil(beamBreak::get), Commands.waitTime(Seconds.of(0.1))),
    State.HOLDING);
```

הפקודות שמעבירים פנימה צריכות לייצג רק *המתנה לאירוע* (`waitUntil`, `waitTime`, וכדומה) - לא
לוגיקת תת-מערכת אמיתית, כיוון שמחזור החיים של פקודת סיום-מצב מונע על ידי מכונת המצבים ולא על ידי
זרימת המתזמן הרגילה.

## `BackgroundCommand`

לא `Command` בעצמה - חריץ קטן שמחזיק פקודה רצה אחת לכל היותר, ומבטל אוטומטית מה שרץ כשמקצים משימה
חדשה. זה מה ש-[`StateMachineBase`](state-machine.md) משתמש בו פנימית ל-`addStateCommand`, אבל זה
שימושי באותה מידה בכל פעם שצריך "תריץ את המשימה הזאת עכשיו, אבל התחלת משימה חדשה תבטל את הישנה":

```java
private final BackgroundCommand idleTask = new BackgroundCommand();

// איפשהו בקוד שלכם:
idleTask.setNewTask(Commands.run(() -> motor.setPercent(0.05)));

// אחר כך, החלפה שלה מבטלת אוטומטית את המשימה הקודמת:
idleTask.setNewTask(Commands.run(() -> motor.setPercent(0)));

idleTask.isRunning(); // האם המשימה הנוכחית עדיין מתוזמנת ולא הסתיימה
idleTask.stop();      // לבטל בלי להחליף
```

`setNewTaskCommand(task)` / `setNewTaskDynamic(supplier)` עוטפים את אותה התנהגות כ-`InstantCommand`
לקישור לטריגר.

## `DetachedCommand`

`InstantCommand` שמעביר פקודה אחרת למתזמן כפקודה עצמאית משלה, במקום להריץ אותה inline. להשתמש בזה
כשפקודה צריכה `requirements` לצורך רצף, אבל העבודה בפועל צריכה להמשיך לרוץ אחרי שהפקודה העוטפת
(או הרצף) מסתיימת:

```java
Commands.sequence(
    prepCommand,
    new DetachedCommand(longRunningBackgroundCommand, subsystem)
    // הרצף "מסתיים" כאן; longRunningBackgroundCommand ממשיך לרוץ באופן עצמאי
);
```

## `LoopCommand`

עוטף פקודה ומפעיל אותה מחדש (`end()` ואז `initialize()`) בכל פעם שהיא מסתיימת, למספר קבוע של
חזרות, תוך שיתוף מופע פקודה אחד בין כל החזרות במקום ליצור מופעים חדשים:

```java
Command blinkTwice = new LoopCommand(
    Commands.sequence(Commands.runOnce(this::toggleLED), Commands.waitTime(Seconds.of(0.1))),
    4 // דלוק, כבה, דלוק, כבה
);
```
