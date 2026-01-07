import React, { useState } from 'react';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import styles from './register.module.css';
import { useHistory } from '@docusaurus/router';

function RegisterPage() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const history = useHistory();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsLoading(true);
    setError(null);

    try {
      console.log('Sending registration request:', { email, password: '***', name });
      const response = await fetch('http://localhost:8000/api/auth/register', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password, name }),
      });

      console.log('Response status:', response.status);
      const data = await response.json();
      console.log('Response data:', data);

      if (!response.ok) {
        throw new Error(data.detail || 'Registration failed');
      }

      // Store the token in localStorage
      localStorage.setItem('authToken', data.access_token);

      // Dispatch a storage event to trigger UI updates in other parts of the app
      window.dispatchEvent(new StorageEvent('storage', {
        key: 'authToken',
        newValue: data.access_token,
        oldValue: null,
        url: window.location.href
      }));

      // Redirect to home page after successful registration
      history.push('/');
    } catch (err: any) {
      console.error('Registration error:', err);
      setError(err.message || 'An error occurred during registration');
    } finally {
      setIsLoading(false);
    }
  };

  const handleGoogleSignup = () => {
    // For local development, we'll use a simulated Google OAuth flow
    // In production, replace this with real OAuth implementation
    const googleAuthUrl = `http://localhost:8000/api/auth/google/signup`;

    // For local development, we'll simulate a successful login
    // In a real app, this would redirect to Google's OAuth server
    const width = 500;
    const height = 600;
    const left = window.screen.width / 2 - width / 2;
    const top = window.screen.height / 2 - height / 2;

    const authWindow = window.open(
      googleAuthUrl,
      'googleAuth',
      `width=${width},height=${height},top=${top},left=${left}`
    );

    // Listen for messages from the authentication window
    window.addEventListener('message', (event) => {
      if (event.origin !== window.location.origin) return;

      if (event.data.type === 'GOOGLE_AUTH_SUCCESS') {
        localStorage.setItem('authToken', event.data.token);

        // Dispatch a storage event to trigger UI updates in other parts of the app
        window.dispatchEvent(new StorageEvent('storage', {
          key: 'authToken',
          newValue: event.data.token,
          oldValue: null,
          url: window.location.href
        }));

        history.push('/');
      } else if (event.data.type === 'GOOGLE_AUTH_ERROR') {
        setError(event.data.message || 'Google authentication failed');
      }
    });
  };

  return (
    <Layout title="Sign Up" description="User registration page">
      <div className={clsx('container', styles.registerContainer)}>
        <div className={clsx('row', styles.registerRow)}>
          <div className="col col--6 col--offset-3">
            <div className={clsx('card', styles.registerCard)}>
              <div className="card__header">
                <h2>Sign Up</h2>
              </div>
              <div className="card__body">
                <form onSubmit={handleSubmit}>
                  <div className="form-group">
                    <label htmlFor="name">Full Name</label>
                    <input
                      type="text"
                      className="form-control"
                      id="name"
                      value={name}
                      onChange={(e) => setName(e.target.value)}
                      placeholder="Enter your full name"
                      required
                    />
                  </div>
                  <div className="form-group">
                    <label htmlFor="email">Email address</label>
                    <input
                      type="email"
                      className="form-control"
                      id="email"
                      value={email}
                      onChange={(e) => setEmail(e.target.value)}
                      placeholder="Enter email"
                      required
                    />
                  </div>
                  <div className="form-group">
                    <label htmlFor="password">Password</label>
                    <input
                      type="password"
                      className="form-control"
                      id="password"
                      value={password}
                      onChange={(e) => setPassword(e.target.value)}
                      placeholder="Password"
                      required
                    />
                  </div>
                  <div className="form-group">
                    <button type="submit" className="button button--primary button--block">
                      Sign Up
                    </button>
                  </div>
                </form>
                <div className={styles.divider}>
                  <span>or</span>
                </div>
                <div className="form-group">
                  <button className="button button--secondary button--block">
                    Continue with Google
                  </button>
                </div>
                <div className={clsx('margin-top--md', styles.loginLink)}>
                  Already have an account? <a href="/login">Log in</a>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default RegisterPage;