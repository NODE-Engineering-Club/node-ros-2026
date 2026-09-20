"""Exactly one organ commands the thrusters.

Two complete paths to the motors exist in this workspace and both subscribe to
``/control/effort``:

* ``actuator_driver`` → MAVROS → Pixhawk → ESCs
* ``pico_bridge`` → serial → Pico → ESCs

They did not collide before only because nothing started ``pico_bridge``. That
is not arbitration, it is an accident, and it ends the moment somebody runs the
node by hand to make the GUI work — which is exactly what deploying this
requires.

``njord.launch.py`` now settles it with one ``motor_path`` argument. This test
checks the property that makes it a guarantee rather than a convention: **no
combination of launch arguments starts both nodes**.

``launch`` is a ROS package and is not importable without a sourced workspace,
so this reads the launch file's source with :mod:`ast` rather than executing
it. That is weaker than launching it — it cannot catch a runtime error — but it
does catch the thing worth catching, which is somebody adding a second way to
enable one of the two nodes and quietly restoring the collision.
"""

from __future__ import annotations

import ast
import itertools
from pathlib import Path

import pytest

LAUNCH_FILE = (
    Path(__file__).resolve().parents[3] / "src" / "bringup" / "launch" / "njord.launch.py"
)

#: The two nodes that must never run together.
RIVALS = ("actuator_driver", "pico_bridge")


def _tree():
    return ast.parse(LAUNCH_FILE.read_text())


def _node_calls(tree):
    """Every ``Node(...)`` call in the file, keyed by its ``executable``."""
    found = {}
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        if not (isinstance(call.func, ast.Name) and call.func.id == "Node"):
            continue
        kwargs = {kw.arg: kw.value for kw in call.keywords}
        executable = kwargs.get("executable")
        if isinstance(executable, ast.Constant):
            found[executable.value] = kwargs
    return found


def _condition_tokens(node_kwargs):
    """A node's condition as an ordered token list.

    ``("lit", text)`` for a string fragment, ``("cfg", name)`` for a
    ``LaunchConfiguration``. Order matters — ``PythonExpression`` concatenates
    its list into one expression — so this walks the list's elements in source
    order rather than using ``ast.walk``, which does not preserve it.
    """
    condition = node_kwargs.get("condition")
    assert condition is not None, "the node has no condition at all"

    expression_list = None
    for sub in ast.walk(condition):
        if (
            isinstance(sub, ast.Call)
            and isinstance(sub.func, ast.Name)
            and sub.func.id == "PythonExpression"
            and sub.args
            and isinstance(sub.args[0], ast.List)
        ):
            expression_list = sub.args[0]
            break

    if expression_list is None:
        # A bare IfCondition(LaunchConfiguration("x")): truthy on "true".
        for sub in ast.walk(condition):
            if _is_launch_configuration(sub):
                return [("cfg", sub.args[0].value)]
        return []

    tokens = []
    for element in expression_list.elts:
        if isinstance(element, ast.Constant) and isinstance(element.value, str):
            tokens.append(("lit", element.value))
        elif _is_launch_configuration(element):
            tokens.append(("cfg", element.args[0].value))
        else:
            raise AssertionError(
                f"unrecognised element in a motor-path condition: "
                f"{ast.dump(element)[:80]}"
            )
    return tokens


def _is_launch_configuration(node):
    return (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "LaunchConfiguration"
        and node.args
        and isinstance(node.args[0], ast.Constant)
    )


def _configurations(tokens):
    return [name for kind, name in tokens if kind == "cfg"]


@pytest.fixture(scope="module")
def nodes():
    found = _node_calls(_tree())
    for name in RIVALS:
        assert name in found, f"{name} is not launched by njord.launch.py at all"
    return found


def test_both_motor_nodes_are_launched_conditionally(nodes):
    """An unconditional node is one that always runs, which is half a collision."""
    for name in RIVALS:
        assert nodes[name].get("condition") is not None, f"{name} runs unconditionally"


def test_both_read_the_same_argument_to_decide(nodes):
    """One input, not two.

    Two independent flags could be set to the same value, and then both nodes
    start. The exclusion holds only because both conditions consult the same
    argument.
    """
    for name in RIVALS:
        configurations = _configurations(_condition_tokens(nodes[name]))
        assert "motor_path" in configurations, (
            f"{name}'s condition does not consult motor_path, so nothing stops "
            f"it running alongside the other motor path"
        )


def test_they_require_opposite_values_of_it(nodes):
    """...and consult it for *different* values."""
    wanted = {}
    for name in RIVALS:
        tokens = _condition_tokens(nodes[name])
        wanted[name] = {
            value
            for kind, text in tokens
            if kind == "lit"
            for value in ("pico", "pixhawk")
            if f"== '{value}'" in text or f"'{value}'" == text.strip()
        }

    assert wanted["actuator_driver"] == {"pixhawk"}, wanted
    assert wanted["pico_bridge"] == {"pico"}, wanted
    assert not (wanted["actuator_driver"] & wanted["pico_bridge"]), (
        "both motor paths accept the same motor_path value"
    )


def test_no_assignment_of_the_arguments_starts_both(nodes):
    """The property itself, by exhaustion rather than by reading.

    Rebuilds each condition as the launch system would — ``PythonExpression``
    concatenates its list into one Python expression — and evaluates it for
    every combination of the arguments involved. If any combination leaves both
    nodes enabled, this fails and names it.
    """
    expressions, arguments = {}, set()
    for name in RIVALS:
        tokens = _condition_tokens(nodes[name])
        expressions[name] = tokens
        arguments |= set(_configurations(tokens))

    values = {
        "motor_path": ["pico", "pixhawk"],
        "enable_control": ["true", "false"],
        "use_sim": ["true", "false"],
    }
    for name in arguments:
        assert name in values, (
            f"{name} appears in a motor-path condition but this test does not "
            f"know its possible values — add them, then re-check the property"
        )

    names = sorted(arguments)
    for combination in itertools.product(*(values[n] for n in names)):
        settings = dict(zip(names, combination))
        enabled = [
            name for name in RIVALS if _evaluate(expressions[name], settings)
        ]
        assert len(enabled) <= 1, (
            f"{settings} starts both motor paths: {enabled}. "
            f"Two nodes would then publish to the thrusters at once."
        )


def _evaluate(tokens, settings):
    """Rebuild a PythonExpression condition and evaluate it.

    ``PythonExpression([...])`` joins its list into one expression string with
    each ``LaunchConfiguration`` replaced by its value. This reproduces that
    join, which is why the tokens are an ordered list.
    """
    if len(tokens) == 1 and tokens[0][0] == "cfg":
        # A bare IfCondition(LaunchConfiguration("x")).
        return settings[tokens[0][1]] == "true"

    expression = "".join(
        text if kind == "lit" else settings[text] for kind, text in tokens
    )
    return bool(eval(expression, {"__builtins__": {}}, {}))  # noqa: S307


def test_the_default_is_the_pico(nodes):
    """The boat flies on the Pico. A default that silently selected the other
    path would be a configuration nobody tested arriving by surprise."""
    tree = _tree()
    for call in ast.walk(tree):
        if (
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
            and call.func.id == "DeclareLaunchArgument"
            and call.args
            and isinstance(call.args[0], ast.Constant)
            and call.args[0].value == "motor_path"
        ):
            defaults = [
                kw.value.value for kw in call.keywords
                if kw.arg == "default_value" and isinstance(kw.value, ast.Constant)
            ]
            assert defaults == ["pico"], defaults
            return
    pytest.fail("motor_path is not declared as a launch argument")
