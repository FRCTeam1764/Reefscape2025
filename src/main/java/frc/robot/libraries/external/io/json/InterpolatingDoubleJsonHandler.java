package frc.robot.libraries.external.io.json;

import com.google.gson.*;
import frc.robot.libraries.external.util.InterpolatingDouble;

import java.lang.reflect.Type;

public final class InterpolatingDoubleJsonHandler implements JsonDeserializer<InterpolatingDouble>, JsonSerializer<InterpolatingDouble> {
    @Override
    public JsonElement serialize(InterpolatingDouble src, Type typeOfSrc, JsonSerializationContext context) {
        return new JsonPrimitive(src.value);
    }

    @Override
    public InterpolatingDouble deserialize(JsonElement json, Type typeOfT, JsonDeserializationContext context) throws JsonParseException {
        return new InterpolatingDouble(json.getAsDouble());
    }
}
