package frc.robot.auto;

import java.lang.annotation.ElementType;
import java.lang.annotation.Target;

@Target(ElementType.TYPE)
public @interface AutoDescription {
    String description() default "No Description";
}
