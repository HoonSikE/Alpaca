package com.example.taxi.di

import android.content.SharedPreferences
import com.example.taxi.data.repository.AuthRepository
import com.example.taxi.data.repository.AuthRepositoryImpl
import com.example.taxi.data.repository.DestinationRepository
import com.example.taxi.data.repository.DestinationRepositoryImpl
import com.google.firebase.auth.FirebaseAuth
import com.google.firebase.firestore.FirebaseFirestore
import com.google.firebase.storage.StorageReference
import com.google.gson.Gson
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
object RepositoryModule {
    @Provides
    @Singleton
    fun provideDestinationRepository(
        database: FirebaseFirestore
    ): DestinationRepository {
        return DestinationRepositoryImpl(database)
    }

    @Provides
    @Singleton
    fun provideAutghRepository(
        database: FirebaseFirestore,
        auth: FirebaseAuth,
        appPreferences: SharedPreferences,
        gson: Gson
    ): AuthRepository {
        return AuthRepositoryImpl(auth,database,appPreferences,gson)
    }
}