# Stage 1: build — install uv and sync dependencies
FROM python:3.12-slim AS build

WORKDIR /app
COPY . /app

RUN pip install uv && \
    uv venv /app/.venv && \
    uv pip install --python /app/.venv/bin/python aiohttp

# Stage 2: runtime — minimal image with venv + static assets
FROM python:3.12-slim AS runtime

WORKDIR /app

COPY --from=build /app/.venv /app/.venv
COPY server.py threejs_scene.html robot_scene.glb ./
COPY stl_files/ ./stl_files/

ENV PATH="/app/.venv/bin:$PATH"

RUN useradd -u 1000 -M -s /sbin/nologin appuser && \
    chown -R appuser /app

USER appuser

EXPOSE 8080

ENTRYPOINT ["python", "server.py"]
CMD ["--host", "0.0.0.0", "--port", "8080"]
