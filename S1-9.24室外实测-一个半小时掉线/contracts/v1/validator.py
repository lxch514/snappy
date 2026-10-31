"""Small dependency-free validator for the JSON Schema subset used by v1.

This is intentionally not a general JSON Schema implementation. It keeps local
tests offline while the schemas remain compatible with full draft-2020-12 tools.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable


class ValidationError(ValueError):
    pass


def _matches_type(value: Any, expected: str) -> bool:
    if expected == "null":
        return value is None
    if expected == "object":
        return isinstance(value, dict)
    if expected == "array":
        return isinstance(value, list)
    if expected == "string":
        return isinstance(value, str)
    if expected == "boolean":
        return isinstance(value, bool)
    if expected == "integer":
        return isinstance(value, int) and not isinstance(value, bool)
    if expected == "number":
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    raise ValidationError(f"unsupported schema type: {expected}")


def _ensure_type(value: Any, expected: str | list[str], path: str) -> None:
    allowed = [expected] if isinstance(expected, str) else expected
    if not any(_matches_type(value, item) for item in allowed):
        raise ValidationError(f"{path}: expected type {allowed}, got {type(value).__name__}")


def validate(value: Any, schema: dict[str, Any], path: str = "$") -> None:
    if "type" in schema:
        _ensure_type(value, schema["type"], path)
    if "const" in schema and value != schema["const"]:
        raise ValidationError(f"{path}: expected constant {schema['const']!r}")
    if "enum" in schema and value not in schema["enum"]:
        raise ValidationError(f"{path}: value {value!r} is not in enum")

    if isinstance(value, dict):
        required: Iterable[str] = schema.get("required", [])
        missing = [key for key in required if key not in value]
        if missing:
            raise ValidationError(f"{path}: missing required fields {missing}")
        properties = schema.get("properties", {})
        if schema.get("additionalProperties") is False:
            extras = sorted(set(value) - set(properties))
            if extras:
                raise ValidationError(f"{path}: unexpected fields {extras}")
        for key, child in value.items():
            if key in properties:
                validate(child, properties[key], f"{path}.{key}")

    if isinstance(value, list) and "items" in schema:
        for index, child in enumerate(value):
            validate(child, schema["items"], f"{path}[{index}]")

    if isinstance(value, str):
        if len(value) < schema.get("minLength", 0):
            raise ValidationError(f"{path}: string is shorter than minLength")
        if "maxLength" in schema and len(value) > schema["maxLength"]:
            raise ValidationError(f"{path}: string is longer than maxLength")

    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if "minimum" in schema and value < schema["minimum"]:
            raise ValidationError(f"{path}: value is below minimum")
        if "maximum" in schema and value > schema["maximum"]:
            raise ValidationError(f"{path}: value is above maximum")


def validate_manifest(contract_root: Path) -> list[tuple[str, bool]]:
    vector_root = contract_root / "test-vectors"
    schema_root = contract_root / "schemas"
    manifest = json.loads((vector_root / "manifest.json").read_text(encoding="utf-8"))
    results: list[tuple[str, bool]] = []
    for case in manifest["cases"]:
        value = json.loads((vector_root / case["file"]).read_text(encoding="utf-8"))
        schema = json.loads((schema_root / case["schema"]).read_text(encoding="utf-8"))
        accepted = True
        try:
            validate(value, schema)
        except ValidationError:
            accepted = False
        results.append((case["file"], accepted))
        if accepted != case["valid"]:
            raise AssertionError(
                f"{case['file']}: expected valid={case['valid']}, got {accepted}"
            )
    return results
