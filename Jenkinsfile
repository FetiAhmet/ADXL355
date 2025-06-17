pipeline {
  agent any
  stages {
    stage('version') {
      steps {
        bat '"C:/Users/fetib/AppData/Local/Microsoft/WindowsApps/python.exe" hello.py'
      }
    }
    stage('hello') {
      steps {
        bat '"C:/Users/fetib/AppData/Local/Microsoft/WindowsApps/python.exe" --version'
      }
    }
  }
}
