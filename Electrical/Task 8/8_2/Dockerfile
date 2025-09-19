# Use official GCC image (comes with g++)
FROM gcc:latest

# Set working directory inside container
WORKDIR /app

# Copy your source code into container
COPY main.cpp .

# Compile the program
RUN g++ -o game main.cpp

# Run it when container starts
CMD ["./game"]

