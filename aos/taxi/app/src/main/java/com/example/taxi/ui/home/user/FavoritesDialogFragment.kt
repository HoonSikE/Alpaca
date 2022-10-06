package com.example.taxi.ui.home.user

import android.graphics.Color
import android.graphics.drawable.ColorDrawable
import android.os.Bundle
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.view.Window
import android.widget.TimePicker.OnTimeChangedListener
import androidx.fragment.app.DialogFragment
import androidx.fragment.app.viewModels
import com.example.taxi.R
import com.example.taxi.data.dto.mypage.Favorites
import com.example.taxi.databinding.DlgFavoritesBinding
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.constant.show
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class FavoritesDialogFragment(var address : String, val confirm : () -> Unit) : DialogFragment() {

    private var _binding: DlgFavoritesBinding? = null
    private val binding get() = _binding!!
    private val userHomeViewModel : UserHomeViewModel by viewModels()

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        _binding = DlgFavoritesBinding.inflate(inflater, container, false)
        dialog?.window?.setBackgroundDrawable(ColorDrawable(Color.TRANSPARENT))
        dialog?.window?.requestFeature(Window.FEATURE_NO_TITLE)
        return binding.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        observerData()
        setOnClickListeners()
    }

    private fun observerData() {
        userHomeViewModel.favorites.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
//                    binding.progressBar.show()
                }
                is UiState.Failure -> {
//                    binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
//                    binding.progressBar.hide()
                    Log.d("실행1 : ", state.data.toString() )
                    var list = state.data as MutableList
                    if(list.size < 2) {
                        userHomeViewModel.deleteFavorites()
                    }else{
                        for(i in 0 until list.size){
                            if(list[i].address == address) {
                                list.removeAt(i)
                                Log.d("실행2 : ", list.toString() )
                                userHomeViewModel.updateFavorites(list)
                            }
                        }
                    }
                }
            }
        }
        userHomeViewModel.deleteFavorites.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
//                    binding.progressBar.show()
                }
                is UiState.Failure -> {
//                    binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
//                    binding.progressBar.hide()
                    Log.d("실행3 : ", state.data.toString() )
                    confirm()
                    dismiss()
                }
            }
        }
        userHomeViewModel.updateFavorites.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
//                    binding.progressBar.show()
                }
                is UiState.Failure -> {
//                    binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
//                    binding.progressBar.hide()
                    Log.d("실행3 : ", state.data.toString() )
                    confirm()
                    dismiss()
                }
            }
        }
    }

    private fun setOnClickListeners() {
        binding.buttonDlgFavoritesFavorites.setOnClickListener { userHomeViewModel.getFavorites() }
        binding.buttonDlgFavoritesCancel.setOnClickListener { dismiss() }
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }

}