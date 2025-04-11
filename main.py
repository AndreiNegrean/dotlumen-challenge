import pytest

TEST_FOLDER_PATH: str = "./tests"
LOG_LEVEL: str = "INFO"


def main() -> None:
    pytest_args: list = [TEST_FOLDER_PATH, f"--log-cli-level={LOG_LEVEL}"]
    pytest.main(pytest_args)


if __name__ == "__main__":
    main()
