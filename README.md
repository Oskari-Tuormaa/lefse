# lefse

**lefse** is a Zephyr module providing modern **C++ wrappers** for various C APIs within the [Zephyr RTOS](https://zephyrproject.org/) framework. It simplifies development by offering safer and more intuitive C++ abstractions while maintaining full compatibility with Zephyr's underlying C APIs.

## ✨ Features

- C++ wrappers for key Zephyr C APIs
- Improved type safety and resource management using RAII patterns
- Seamless integration with existing Zephyr projects
- Lightweight and efficient, designed for embedded systems

## ⚡ Supported APIs

**lefse** currently offers C++ wrappers for the following Zephyr APIs:

- **Timer** – Easy-to-use abstractions for managing system timers.  
- **UART** – Comprehensive UART support:  
  - **Polling** mode  
  - **Interrupt-driven** mode  
  - **Asynchronous** mode  
- **GPIO** – Simplified GPIO configuration and event handling.  

Additional API wrappers are planned for future releases.

## 📦 Getting Started

### Prerequisites

- [Zephyr RTOS](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) (v3.7.0 or later)  
- C++20 compatible compiler  

### Adding lefse to Your Zephyr Manifest

To integrate **lefse** into your Zephyr project, add it as a module in your `west.yml` manifest file:

```yaml
manifest:
  projects:
    - name: lefse
      repo: https://github.com/Oskari-Tuormaa/lefse.git
      revision: main  # or specify a tag/commit
      path: modules/lefse
```

Then, run:

```bash
west update
```

### Using lefse in Your Project

Simply include **lefse** headers in your C++ code:

```cpp
#include <lefse/uart.hpp>

lefse::uart uart { DEVICE_DT_GET(DT_NODELABEL(uart0)) };

int main() {
    uart.write("Hello, Zephyr!\n");
    return 0;
}
```

<!--## 📚 Documentation-->

<!--- [API Reference](docs/API.md) *(To be added)*  -->
<!--- [Usage Examples](examples/)  -->

<!--## 🤝 Contributing-->

<!--Contributions are welcome! Feel free to open issues or submit pull requests. See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.-->

## 📄 License

This project is licensed under the [MIT License](LICENSE).
