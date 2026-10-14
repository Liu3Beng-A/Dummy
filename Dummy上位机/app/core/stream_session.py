"""Deprecated diagnostic-only import. It cannot execute planned point motion."""
from app.core.diagnostic_session import DiagnosticSession

StreamSession = DiagnosticSession

__all__ = ['DiagnosticSession', 'StreamSession']
