# Contributing to Byte

Thank you for your interest in contributing to Byte! This document provides guidelines and information for contributors.

## 🤝 How to Contribute

### Types of Contributions

We welcome contributions in the following areas:

- **Bug Reports** - Help us identify and fix issues
- **Feature Requests** - Suggest new features and improvements
- **Code Contributions** - Submit pull requests with code changes
- **Documentation** - Improve or expand documentation
- **Examples** - Create new examples and tutorials
- **Testing** - Add tests or improve test coverage

## 🚀 Getting Started

### Prerequisites

- Python 3.7 or higher
- Git
- Raspberry Pi (for hardware testing)
- Basic knowledge of robotics and Python

### Development Setup

1. **Fork the repository**
   ```bash
   git clone https://github.com/your-username/byte.git
   cd byte
   ```

2. **Create a virtual environment**
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install development dependencies**
   ```bash
   pip install -e ".[dev]"
   ```

4. **Install pre-commit hooks**
   ```bash
   pre-commit install
   ```

## 📝 Development Guidelines

### Code Style

We follow PEP 8 with some modifications:

- **Line length**: 88 characters (Black default)
- **Docstrings**: Google style
- **Type hints**: Required for public APIs
- **Imports**: Organized with isort

### Code Formatting

We use automated tools for code formatting:

```bash
# Format code
black byte/ samples/ tests/

# Sort imports
isort byte/ samples/ tests/

# Check code style
flake8 byte/ samples/ tests/
```

### Testing

Write tests for new features and ensure all tests pass:

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=byte --cov-report=html

# Run specific test file
pytest tests/test_robotdog.py
```

### Documentation

- Update docstrings for new functions and classes
- Add examples in the documentation
- Update README.md if needed
- Follow the existing documentation style

## 🔧 Development Workflow

### 1. Create a Feature Branch

```bash
git checkout -b feature/your-feature-name
```

### 2. Make Your Changes

- Write your code following the style guidelines
- Add tests for new functionality
- Update documentation as needed

### 3. Test Your Changes

```bash
# Run tests
pytest

# Check code style
flake8 byte/ samples/ tests/

# Format code
black byte/ samples/ tests/
```

### 4. Commit Your Changes

```bash
git add .
git commit -m "feat: Add new feature description"
```

Use conventional commit messages:
- `feat:` for new features
- `fix:` for bug fixes
- `docs:` for documentation changes
- `test:` for test additions
- `refactor:` for code refactoring

### 5. Push and Create Pull Request

```bash
git push origin feature/your-feature-name
```

Then create a pull request on GitHub.

## 📋 Pull Request Guidelines

### Before Submitting

- [ ] Code follows style guidelines
- [ ] Tests pass and coverage is maintained
- [ ] Documentation is updated
- [ ] No merge conflicts
- [ ] Commit messages are clear and descriptive

### Pull Request Template

```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Documentation update
- [ ] Test addition
- [ ] Other (please describe)

## Testing
- [ ] All tests pass
- [ ] New tests added for new functionality
- [ ] Manual testing completed

## Checklist
- [ ] Code follows style guidelines
- [ ] Self-review completed
- [ ] Documentation updated
- [ ] No breaking changes (or documented if necessary)
```

## 🐛 Bug Reports

When reporting bugs, please include:

1. **Environment information**
   - Python version
   - Operating system
   - Hardware setup (Raspberry Pi model, etc.)

2. **Steps to reproduce**
   - Clear, step-by-step instructions
   - Minimal code example if applicable

3. **Expected vs actual behavior**
   - What you expected to happen
   - What actually happened

4. **Additional information**
   - Error messages and stack traces
   - Screenshots if relevant
   - Log files if available

## 💡 Feature Requests

When requesting features:

1. **Describe the feature**
   - What it should do
   - Why it's needed
   - How it would be used

2. **Provide context**
   - Use cases and examples
   - Related existing features
   - Potential implementation approach

3. **Consider impact**
   - Backward compatibility
   - Performance implications
   - Maintenance requirements

## 🧪 Testing Guidelines

### Writing Tests

- Write tests for all new functionality
- Use descriptive test names
- Test both success and failure cases
- Mock hardware dependencies when appropriate

### Test Structure

```python
def test_feature_name():
    """Test description."""
    # Arrange
    dog = RobotDog()
    
    # Act
    result = dog.do_action('test_action')
    
    # Assert
    assert result is not None
    assert result.status == 'success'
```

### Hardware Testing

For hardware-related changes:
- Test on actual Raspberry Pi hardware
- Verify sensor readings and servo movements
- Test edge cases and error conditions

## 📚 Documentation Guidelines

### Code Documentation

- Use Google-style docstrings
- Include type hints
- Provide usage examples
- Document exceptions and edge cases

### User Documentation

- Write clear, concise instructions
- Include code examples
- Add troubleshooting sections
- Keep documentation up to date

## 🔒 Security

- Don't commit sensitive information (API keys, passwords)
- Use environment variables for configuration
- Follow security best practices
- Report security issues privately

## 🎯 Areas for Contribution

### High Priority

- **Bug fixes** - Especially hardware-related issues
- **Performance improvements** - Optimize servo control and sensor reading
- **Error handling** - Improve robustness and user feedback
- **Testing** - Add more comprehensive tests

### Medium Priority

- **New features** - Additional robot behaviors and capabilities
- **Documentation** - Improve guides and examples
- **Examples** - Create new tutorials and demonstrations
- **Tools** - Development and debugging utilities

### Low Priority

- **Code refactoring** - Improve code organization and structure
- **Optimization** - Performance tuning and memory usage
- **UI improvements** - Better user interfaces and controls

## 🤝 Community Guidelines

### Be Respectful

- Treat all contributors with respect
- Provide constructive feedback
- Be patient with newcomers
- Help others learn and grow

### Communication

- Use clear, professional language
- Ask questions when unsure
- Provide context for suggestions
- Be open to different approaches

### Recognition

- Credit contributors in release notes
- Highlight significant contributions
- Thank contributors for their work
- Celebrate project milestones

## 📞 Getting Help

If you need help contributing:

1. **Check existing documentation**
2. **Search existing issues and discussions**
3. **Ask questions in GitHub Discussions**
4. **Join our community channels**

## 📄 License

By contributing to Byte, you agree that your contributions will be licensed under the MIT License.

---

Thank you for contributing to Byte! Your help makes this project better for everyone. 🐕🤖 