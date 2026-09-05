# כניסה דיגיטלית מתועדת

`LoggedDigitalInput` עוטף חיישן דיגיטלי פשוט - מתג הגבלה, beam break, וכדומה - ומוסיף תיעוד
אוטומטי דרך `NinjasLogger` וברירת מחדל מובנית להפעלה/כיבוי, כך שחיישן אופציונלי שאולי לא מחווט
ברובוט מסוים לא צריך בדיקת `if (hasSensor)` בכל נקודת קריאה.

## הגדרה

```java
LoggedDigitalInput beamBreak = new LoggedDigitalInput(
    "Indexer/BeamBreak",
    0,                          // פורט DIO
    kHasBeamBreak,              // מופעל?
    /* disabledValue = */ false,
    /* inverted = */ true,
    Robot.isReal() ? new LoggedDigitalInputIOReal() : new LoggedDigitalInputIOSim()
);
```

אם `enabled` הוא `false`, החומרה כלל לא נגעת - `io` לא בשימוש, ו-`get()` תמיד מחזיר את
`disabledValue` במקום. זה הדפוס לחיישן שקיים בחלק מהרובוטים בקבוצה אבל לא בכולם: משאירים את נקודות
הקריאה זהות ומחליפים קבוע אחד לכל רובוט.

## שימוש

```java
@Override
public void periodic() {
    beamBreak.periodic(); // בודק חומרה (אם מופעל) ומתעד את התוצאה
}

public boolean hasPiece() {
    return beamBreak.get();
}
```

זה משתלב באופן טבעי עם [מכונות מצבים](state-machine.md): beam break הוא תנאי `addStateEnd` נפוץ
(`addStateEnd(State.INTAKING, beamBreak::get, State.HOLDING)`).
