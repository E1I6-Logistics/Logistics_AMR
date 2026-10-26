"""Small, explicit terminal prompts for supervised physical experiments."""


class OperatorAbort(RuntimeError):
    """Raised when the operator intentionally stops an experiment."""


def confirm(prompt, default=None):
    """Ask a yes/no question."""
    suffix = " [y/n] "
    if default is True:
        suffix = " [Y/n] "
    elif default is False:
        suffix = " [y/N] "
    while True:
        value = input(prompt + suffix).strip().lower()
        if not value and default is not None:
            return default
        if value in ("y", "yes", "예", "네"):
            return True
        if value in ("n", "no", "아니오"):
            return False
        if value in ("q", "quit"):
            raise OperatorAbort("Operator requested stop")
        print("y 또는 n을 입력하세요. 종료하려면 q를 입력하세요.")


def wait_for_enter(prompt):
    """Wait for operator confirmation while supporting an intentional stop."""
    value = input(prompt + " [ENTER, 종료=q] ").strip().lower()
    if value in ("q", "quit"):
        raise OperatorAbort("Operator requested stop")


def optional_nonnegative_float(prompt):
    """Read a non-negative float; blank means unavailable."""
    while True:
        value = input(prompt + " [미측정=ENTER] ").strip()
        if value == "":
            return None
        try:
            number = float(value)
            if number < 0.0:
                raise ValueError
            return number
        except ValueError:
            print("0 이상의 숫자 또는 ENTER를 입력하세요.")


def optional_percentage(prompt):
    """Read a percentage from 0 through 100; blank means unavailable."""
    while True:
        value = input(prompt + " [미측정=ENTER] ").strip()
        if value == "":
            return None
        try:
            number = float(value)
            if not 0.0 <= number <= 100.0:
                raise ValueError
            return number
        except ValueError:
            print("0~100 사이 숫자 또는 ENTER를 입력하세요.")


def choice(prompt, options, default=None):
    """Read one normalized value from a fixed set."""
    option_text = "/".join(options)
    while True:
        value = input(f"{prompt} [{option_text}] ").strip().lower()
        if value == "" and default is not None:
            return default
        if value in options:
            return value
        if value in ("q", "quit"):
            raise OperatorAbort("Operator requested stop")
        print(f"다음 중 하나를 입력하세요: {option_text}")


def optional_text(prompt):
    """Read optional free text."""
    return input(prompt + " [없음=ENTER] ").strip()
