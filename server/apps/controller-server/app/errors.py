from __future__ import annotations


class ApiError(Exception):
    def __init__(self, status_code: int, message: str, error: str | None = None):
        super().__init__(message)
        self.status_code = status_code
        self.message = message
        self.error = error or {
            400: "Bad Request",
            401: "Unauthorized",
            404: "Not Found",
        }.get(status_code, "Error")
